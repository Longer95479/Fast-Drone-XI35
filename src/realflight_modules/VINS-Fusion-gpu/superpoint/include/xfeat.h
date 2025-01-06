#ifndef XFEAT_H_
#define XFEAT_H_

#include <string>
#include <memory>
#include <Eigen/Core>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <opencv2/opencv.hpp>

#include "3rdparty/tensorrtbuffer/include/buffers.h"
#include "read_configs.h"

using tensorrt_buffer::TensorRTUniquePtr;

class Xfeat {
public:
    explicit Xfeat(const XfeatConfig &sxfeat_config);

    bool build();

    bool infer(const cv::Mat &image_, Eigen::Matrix<float, 67, Eigen::Dynamic> &features);

    bool infer_origin(const cv::Mat &image_, float* heatmap, float* descriptors);

    void save_engine();

    bool deserialize_engine();

    XfeatConfig xfeat_config_;
private:
    int input_width;
    int input_height;
    int resized_width;
    int resized_height;
    float w_scale;
    float h_scale; 

    nvinfer1::Dims input_dims_{};
    nvinfer1::Dims desc_dims_{};
    nvinfer1::Dims score_dims_{};
    nvinfer1::Dims relimap_dims_{};
    std::shared_ptr<nvinfer1::ICudaEngine> engine_;
    std::shared_ptr<nvinfer1::IExecutionContext> context_;

    bool construct_network(TensorRTUniquePtr<nvinfer1::IBuilder> &builder,
                           TensorRTUniquePtr<nvinfer1::INetworkDefinition> &network,
                           TensorRTUniquePtr<nvinfer1::IBuilderConfig> &config,
                           TensorRTUniquePtr<nvonnxparser::IParser> &parser) const;

    bool process_input(const tensorrt_buffer::BufferManager &buffers, const cv::Mat &image);

    bool process_output(const tensorrt_buffer::BufferManager &buffers, Eigen::Matrix<float, 67, Eigen::Dynamic> &features);

    bool keypoints_decoder(const float* scores, const float* descriptors, Eigen::Matrix<float, 67, Eigen::Dynamic> &features);

    std::vector<int> sort_indexes(std::vector<float> &data);
    int clip(int val, int max);

    void detect_point(const float* heat_map, Eigen::Matrix<float, 67, Eigen::Dynamic>& features, int h, int w, float threshold, int border, int top_k);
    void extract_descriptors(const float *descriptors, Eigen::Matrix<float, 67, Eigen::Dynamic> &features, int h, int w, int s);

    std::vector<std::pair<int, cv::Point2f>> nms_process(const std::vector<cv::Point2f>& pts, const std::vector<int>& sorted_idx, float dist_thresh);
};

typedef std::shared_ptr<Xfeat> XfeatPtr;

#endif
