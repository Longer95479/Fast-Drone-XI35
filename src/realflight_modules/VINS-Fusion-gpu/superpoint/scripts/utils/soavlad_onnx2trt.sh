trtexec \
--onnx=/home/nx05/soa-vlad/soa_attnvlad.onnx \
--minShapes=input:1x64x60x80 \
--optShapes=input:1x64x60x80 \
--maxShapes=input:1x64x60x80 \
--workspace=4096 \
--fp16 \
--saveEngine=soavlad_layer.engine
