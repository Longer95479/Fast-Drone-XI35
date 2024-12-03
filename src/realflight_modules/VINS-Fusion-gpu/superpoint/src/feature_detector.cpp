#include <opencv2/opencv.hpp>

#include "plnet.h"
#include "feature_detector.h"
#include "utils.h"

FeatureDetector::FeatureDetector(const PLNetConfig& plnet_config) : _plnet_config(plnet_config){
	if(_plnet_config.use_superpoint == 1){
		SuperPointConfig superpoint_config;
		superpoint_config.max_keypoints = plnet_config.max_keypoints;
		superpoint_config.keypoint_threshold = plnet_config.keypoint_threshold;
		superpoint_config.remove_borders = plnet_config.remove_borders;
		superpoint_config.dist_thresh = plnet_config.dist_thresh;
		superpoint_config.dla_core = -1;

		superpoint_config.input_tensor_names.push_back("input");
		superpoint_config.output_tensor_names.push_back("scores");
		superpoint_config.output_tensor_names.push_back("descriptors");

		superpoint_config.onnx_file = plnet_config.superpoint_onnx;
		superpoint_config.engine_file = plnet_config.superpoint_engine;

		_superpoint = std::shared_ptr<SuperPoint>(new SuperPoint(superpoint_config));
		if (!_superpoint->build()){
		std::cout << "Error in SuperPoint building" << std::endl;
		exit(0);
		}
	}
	else if(_plnet_config.use_superpoint == 2){
		XfeatConfig xfeat_config;
		xfeat_config.max_keypoints = plnet_config.max_keypoints;
		xfeat_config.keypoint_threshold = plnet_config.keypoint_threshold;
		xfeat_config.remove_borders = plnet_config.remove_borders;
		xfeat_config.dist_thresh = plnet_config.dist_thresh;
		xfeat_config.dla_core = -1;

		xfeat_config.input_tensor_names.push_back("input");
		xfeat_config.output_tensor_names.push_back("output_feats");
		xfeat_config.output_tensor_names.push_back("output_scores");
		xfeat_config.output_tensor_names.push_back("output_reliability");

		xfeat_config.onnx_file = plnet_config.xfeat_onnx;
		xfeat_config.engine_file = plnet_config.xfeat_engine;
		
		_xfeat = std::shared_ptr<Xfeat>(new Xfeat(xfeat_config));
		if (!_xfeat->build()){
		std::cout << "Error in Xfeat building" << std::endl;
		exit(0);
		}
	}
	#if 0
	_plnet = std::shared_ptr<PLNet>(new PLNet(_plnet_config));
	if (!_plnet->build()){
		std::cout << "Error in FeatureDetector building" << std::endl;
		// exit(0);
	}
	#endif
}
//直接获取heatmap和描述子特征图，heatmap和descriptors需要提前分配好内存
bool FeatureDetector::DetectHDUseXfeat(cv::Mat& image, float* heatmap, float* descriptors)
{
	bool good_infer = false;
	if(_plnet_config.use_superpoint == 2)
	{
		good_infer = _xfeat->infer_origin(image, heatmap, descriptors);
	}
	if(!good_infer)
		std::cout << "Failed when extracting heatmap, descriptors !" << std::endl;
	return good_infer; 
}

bool FeatureDetector::DetectUseXfeat(cv::Mat& image, Eigen::Matrix<float, 67, Eigen::Dynamic> &features){
	bool good_infer = false;
	if(_plnet_config.use_superpoint == 2){
		good_infer = _xfeat->infer(image, features);
	}
	if(!good_infer){
		std::cout << "Failed when extracting point features !" << std::endl;
	}
	return good_infer; 
}

//features[0]:score features[1~2]:(x,y) features[3~259]:desc
bool FeatureDetector::Detect(cv::Mat& image, Eigen::Matrix<float, 259, Eigen::Dynamic> &features){
	bool good_infer = false;
	if(_plnet_config.use_superpoint){
		good_infer = _superpoint->infer(image, features);
	}else{
		std::vector<Eigen::Vector4d> lines;
		good_infer = Detect(image, features, lines);
	}


	if(!good_infer){
		std::cout << "Failed when extracting point features !" << std::endl;
	}
	return good_infer; 
}

bool FeatureDetector::Detect(cv::Mat& image, Eigen::Matrix<float, 259, Eigen::Dynamic> &features, 
		std::vector<Eigen::Vector4d>& lines){
	Eigen::Matrix<float, 259, Eigen::Dynamic> junctions;
	bool good_infer = _plnet->infer(image, features, lines, junctions);
	if(!good_infer){
		std::cout << "Failed when extracting point features !" << std::endl;
	}
	return good_infer; 
}

bool FeatureDetector::Detect(cv::Mat& image, Eigen::Matrix<float, 259, Eigen::Dynamic> &features, 
		std::vector<Eigen::Vector4d>& lines, Eigen::Matrix<float, 259, Eigen::Dynamic>& junctions){
	bool good_infer = _plnet->infer(image, features, lines, junctions, true);
	if(!good_infer){
		std::cout << "Failed when extracting point features !" << std::endl;
	}
	return good_infer; 
}

bool FeatureDetector::Detect(cv::Mat& image_left, cv::Mat& image_right, 
		Eigen::Matrix<float, 259, Eigen::Dynamic> & left_features, 
		Eigen::Matrix<float, 259, Eigen::Dynamic> & right_features){
	bool good_infer_left = Detect(image_left, left_features);
	bool good_infer_right = Detect(image_right, right_features);
	bool good_infer = good_infer_left & good_infer_right;
	if(!good_infer){
		std::cout << "Failed when extracting point features !" << std::endl;
	}
	return good_infer; 
}

bool FeatureDetector::Detect(cv::Mat& image_left, cv::Mat& image_right, 
		Eigen::Matrix<float, 259, Eigen::Dynamic> & left_features, 
		Eigen::Matrix<float, 259, Eigen::Dynamic> & right_features, 
		std::vector<Eigen::Vector4d>& left_lines, 
		std::vector<Eigen::Vector4d>& right_lines){
	bool good_infer_left = Detect(image_left, left_features, left_lines);
	bool good_infer_right = Detect(image_right, right_features, right_lines);
	bool good_infer = good_infer_left & good_infer_right;
	if(!good_infer){
		std::cout << "Failed when extracting point features !" << std::endl;
	}
	return good_infer; 
}

bool FeatureDetector::Detect(cv::Mat& image_left, cv::Mat& image_right, Eigen::Matrix<float, 259, Eigen::Dynamic> & left_features, 
		Eigen::Matrix<float, 259, Eigen::Dynamic> & right_features, std::vector<Eigen::Vector4d>& left_lines, 
		std::vector<Eigen::Vector4d>& right_lines, Eigen::Matrix<float, 259, Eigen::Dynamic>& junctions){
	bool good_infer_left = Detect(image_left, left_features, left_lines, junctions);
	bool good_infer_right = Detect(image_right, right_features, right_lines);

	bool good_infer = good_infer_left & good_infer_right;
	if(!good_infer){
		std::cout << "Failed when extracting point features !" << std::endl;
	}
	return good_infer; 
}

void FeatureDetector::prewarmInference()
{
	std::srand(static_cast<unsigned int>(std::time(0)));
	int height = 480, width = 640;
	cv::Mat dummyImage(height, width, CV_8UC1);
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
			dummyImage.at<uchar>(y, x) = static_cast<uchar>(std::rand() % 256);
    }
	Eigen::Matrix<float, 259, Eigen::Dynamic> features;
	Detect(dummyImage, features);
	std::cout << "prewarm for superpoint completed!" << std::endl;
}

int FeatureDetector::getDetectNetworkType()
{
	return _plnet_config.use_superpoint;
}

double FeatureDetector::getDetectPointThreshold()
{
	if(getDetectNetworkType() == 1)
		return _superpoint->super_point_config_.keypoint_threshold;
	else if(getDetectNetworkType() == 2)
		return _xfeat->xfeat_config_.keypoint_threshold;
	else
		return 0;
}

void FeatureDetector::setDetectPointThreshold(double new_thresh)
{
	if(getDetectNetworkType() == 1)
		_superpoint->super_point_config_.keypoint_threshold = new_thresh;
	else if(getDetectNetworkType() == 2)
		_xfeat->xfeat_config_.keypoint_threshold = new_thresh;
}