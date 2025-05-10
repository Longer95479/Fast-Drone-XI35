trtexec --onnx=plnet_s0.onnx \
        --minShapes=input:1x1x512x512 \
        --optShapes=input:1x1x512x512 \
        --maxShapes=input:1x1x512x512 \
        --workspace=8192 \
        --saveEngine=plnet_s0.engine \
        --fp16 