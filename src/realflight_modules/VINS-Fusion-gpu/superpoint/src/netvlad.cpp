#include "netvlad.h"

#include "tic_toc.h"

using namespace tensorrt_log;
using namespace tensorrt_buffer;

bool NetVLAD::process_input(const tensorrt_buffer::BufferManager &buffer,
                            const float *local_feats_desc) {
  if (local_feats_desc == nullptr) return false;

  auto *host_input_buffer =
      static_cast<float *>(buffer.getHostBuffer(config_.input_tensor_names[0]));
  memcpy(host_input_buffer, local_feats_desc,
         buffer.size(config_.input_tensor_names[0]));
  return true;
}

bool NetVLAD::process_output(const tensorrt_buffer::BufferManager &buffer,
                             Eigen::VectorXf &vlad_feat) {
  auto *host_output_buffer = static_cast<float *>(
      buffer.getHostBuffer(config_.output_tensor_names[0]));
  int vlad_len = output_dims_.d[1];
  vlad_feat = Eigen::Map<Eigen::VectorXf>(host_output_buffer, vlad_len);
  return true;
}

bool NetVLAD::infer(const float *local_feats_desc, int local_feats_num,
                    Eigen::VectorXf &vlad_feat) {
  if (!context_) {
    context_ = TensorRTUniquePtr<nvinfer1::IExecutionContext>(
        engine_->createExecutionContext());
    if (!context_) return false;
  }

  const int input_index =
      engine_->getBindingIndex(config_.input_tensor_names[0].c_str());
  const int output_index =
      engine_->getBindingIndex(config_.output_tensor_names[0].c_str());
  context_->setBindingDimensions(input_index,
                                 nvinfer1::Dims3(1, 64, local_feats_num));
  intput_dims_ = context_->getBindingDimensions(input_index);
  output_dims_ = context_->getBindingDimensions(output_index);

  BufferManager buffers(engine_, 0, context_.get());

  if (!process_input(buffers, local_feats_desc)) {
    return false;
  }

  buffers.copyInputToDevice();

  TicToc tic_e;
  bool status = context_->executeV2(buffers.getDeviceBindings().data());
  if (!status) {
    return false;
  }
  ROS_DEBUG("NetVLAD: infer cost %f ms.", tic_e.toc());

  buffers.copyOutputToHost();

  if (!process_output(buffers, vlad_feat)) {
    return false;
  }

  return true;
}

bool NetVLAD::build() {
  if (deserialize_engine()) {
    const int input_index =
        engine_->getBindingIndex(config_.input_tensor_names[0].c_str());
    const int output_index =
        engine_->getBindingIndex(config_.output_tensor_names[0].c_str());

    intput_dims_ = engine_->getBindingDimensions(input_index);
    output_dims_ = engine_->getBindingDimensions(output_index);
    return true;
  }
  std::cout << "deserialize netvlad engine failed, will build it at runtime."
            << std::endl;
  auto builder = TensorRTUniquePtr<nvinfer1::IBuilder>(
      nvinfer1::createInferBuilder(gLogger.getTRTLogger()));
  const auto explicit_batch =
      1U << static_cast<uint32_t>(
          nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = TensorRTUniquePtr<nvinfer1::INetworkDefinition>(
      builder->createNetworkV2(explicit_batch));
  if (!network) {
    return false;
  }
  auto config = TensorRTUniquePtr<nvinfer1::IBuilderConfig>(
      builder->createBuilderConfig());
  if (!config) {
    return false;
  }
  auto parser = TensorRTUniquePtr<nvonnxparser::IParser>(
      nvonnxparser::createParser(*network, gLogger.getTRTLogger()));
  if (!parser) {
    return false;
  }
  auto profile = builder->createOptimizationProfile();
  if (!profile) {
    return false;
  }
  profile->setDimensions(config_.input_tensor_names[0].c_str(),
                         nvinfer1::OptProfileSelector::kMIN,
                         nvinfer1::Dims3(1, 64, 1));
  profile->setDimensions(config_.input_tensor_names[0].c_str(),
                         nvinfer1::OptProfileSelector::kOPT,
                         nvinfer1::Dims3(1, 64, 4800));
  profile->setDimensions(config_.input_tensor_names[0].c_str(),
                         nvinfer1::OptProfileSelector::kMAX,
                         nvinfer1::Dims3(1, 64, 9600));

  config->addOptimizationProfile(profile);

  auto constructed = construct_network(builder, network, config, parser);
  if (!constructed) {
    return false;
  }

  auto profile_stream = makeCudaStream();
  if (!profile_stream) {
    return false;
  }
  config->setProfileStream(*profile_stream);

  TensorRTUniquePtr<nvinfer1::IHostMemory> plan{
      builder->buildSerializedNetwork(*network, *config)};
  if (!plan) {
    return false;
  }

  TensorRTUniquePtr<nvinfer1::IRuntime> runtime{
      nvinfer1::createInferRuntime(gLogger.getTRTLogger())};
  if (!runtime) {
    return false;
  }

  engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
      runtime->deserializeCudaEngine(plan->data(), plan->size()));
  if (!engine_) {
    return false;
  }

  save_engine();
  intput_dims_ = network->getInput(0)->getDimensions();
  output_dims_ = network->getOutput(0)->getDimensions();
  return true;
}

bool NetVLAD::construct_network(
    TensorRTUniquePtr<nvinfer1::IBuilder> &builder,
    TensorRTUniquePtr<nvinfer1::INetworkDefinition> &network,
    TensorRTUniquePtr<nvinfer1::IBuilderConfig> &config,
    TensorRTUniquePtr<nvonnxparser::IParser> &parser) const {
  auto parsed =
      parser->parseFromFile(config_.onnx_file.c_str(),
                            static_cast<int>(gLogger.getReportableSeverity()));
  if (!parsed) {
    return false;
  }
  config->setFlag(nvinfer1::BuilderFlag::kFP16);
  enableDLA(builder.get(), config.get(), -1);
  return true;
}

void NetVLAD::save_engine() {
  if (config_.engine_file.empty()) return;
  if (engine_ != nullptr) {
    nvinfer1::IHostMemory *data = engine_->serialize();
    std::ofstream file(config_.engine_file, std::ios::binary);
    if (!file) return;
    file.write(reinterpret_cast<const char *>(data->data()), data->size());
  }
}

bool NetVLAD::deserialize_engine() {
  initLibNvInferPlugins(&gLogger, "");
  std::ifstream file(config_.engine_file.c_str(), std::ios::binary);
  if (file.is_open()) {
    file.seekg(0, std::ifstream::end);
    size_t size = file.tellg();
    file.seekg(0, std::ifstream::beg);
    char *model_stream = new char[size];
    file.read(model_stream, size);
    file.close();
    nvinfer1::IRuntime *runtime = nvinfer1::createInferRuntime(gLogger);
    // if (runtime == nullptr) return false;
    if (runtime == nullptr) {
      delete[] model_stream;
      return false;
    }
    engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
        runtime->deserializeCudaEngine(model_stream, size));
    delete[] model_stream;
    if (engine_ == nullptr) return false;
    std::cout << "deserialize netvlad engine successfully!" << std::endl;
    return true;
  }
  return false;
}
