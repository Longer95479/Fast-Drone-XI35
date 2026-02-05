trtexec \
--onnx=/home/nx05/xfeat-vlad/netvlad_layer.onnx \
--minShapes=input:1x64x1 \
--optShapes=input:1x64x4800 \
--maxShapes=input:1x64x9600 \
--workspace=4096 \
--fp16 \
--saveEngine=netvlad_layer.engine