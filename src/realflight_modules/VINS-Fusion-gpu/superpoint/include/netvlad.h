#pragma once

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <eigen3/Eigen/Dense>

#include "3rdparty/tensorrtbuffer/include/buffers.h"
#include "read_configs.h"

using tensorrt_buffer::TensorRTUniquePtr;

class NetVLAD {
 public:
  explicit NetVLAD(const NetVLADConfig& config) : config_(config) {}

  bool build();
  bool infer(const float* local_feats_desc, int local_feats_num,
             Eigen::VectorXf& vlad_feat);

 private:
  void save_engine();
  bool deserialize_engine();
  bool construct_network(
      TensorRTUniquePtr<nvinfer1::IBuilder>& builder,
      TensorRTUniquePtr<nvinfer1::INetworkDefinition>& network,
      TensorRTUniquePtr<nvinfer1::IBuilderConfig>& config,
      TensorRTUniquePtr<nvonnxparser::IParser>& parser) const;
  bool process_input(const tensorrt_buffer::BufferManager& buffer,
                     const float* local_feats_desc);
  bool process_output(const tensorrt_buffer::BufferManager& buffer,
                      Eigen::VectorXf& vlad_feat);

  NetVLADConfig config_;

  nvinfer1::Dims intput_dims_;
  nvinfer1::Dims output_dims_;

  std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IExecutionContext> context_;
};
typedef std::shared_ptr<NetVLAD> NetVLADPtr;