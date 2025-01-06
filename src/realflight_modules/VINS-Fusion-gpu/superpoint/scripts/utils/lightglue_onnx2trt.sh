trtexec \
--onnx=superpoint_lightglue.plugin.onnx \
--plugins=./trt_plugins/libcustom_layernorm.so \
--minShapes=keypoints_0:1x1x2,keypoints_1:1x1x2,descriptors_0:1x1x256,descriptors_1:1x1x256 \
--optShapes=keypoints_0:1x512x2,keypoints_1:1x512x2,descriptors_0:1x512x256,descriptors_1:1x512x256 \
--maxShapes=keypoints_0:1x1024x2,keypoints_1:1x1024x2,descriptors_0:1x1024x256,descriptors_1:1x1024x256 \
--workspace=4096 \
--fp16 \
--saveEngine=superpoint_lightglue.engine