import tensorrt as trt

# 加载TensorRT引擎
def load_engine(engine_path):
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, 'rb') as f:
        engine_data = f.read()
    engine = trt.Runtime(TRT_LOGGER).deserialize_cuda_engine(engine_data)
    return engine

# 打印模型信息
def print_model_info(engine):
    # 打印输入和输出的名称、形状和数据类型
    for i in range(engine.num_io_tensors):
        name = engine.get_tensor_name(i)
        if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
            is_input = True
        else:
            is_input = False
        shape = engine.get_tensor_shape(name)
        type = trt.nptype(engine.get_tensor_dtype(name))
        print(f"Tensor {i}: {'Input' if is_input else 'Output'}, Name: {name}, Shape: {shape}, Type: {type}")

def main():
    engine_path = 'netvlad_layer.engine'
    engine = load_engine(engine_path)
    print_model_info(engine)
 
if __name__ == "__main__":
    main()