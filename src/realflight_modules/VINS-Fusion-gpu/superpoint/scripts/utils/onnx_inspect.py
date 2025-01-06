import numpy as np
import onnx
import onnxruntime
from time import time

# 加载 ONNX 模型
model_path = "superpoint_lightglue.onnx"
model = onnx.load(model_path)
onnx.checker.check_model(model)

print("Model IR Version:", model.ir_version)
print("Producer Name:", model.producer_name)
print("Opset Version:", model.opset_import[0].version)

# 打印模型输入信息
print("Inputs:")
for input_tensor in model.graph.input:
    print(f"Name: {input_tensor.name}")
    print(f"Type: {input_tensor.type.tensor_type.elem_type}")
    shape = [dim.dim_value for dim in input_tensor.type.tensor_type.shape.dim]
    print(f"Shape: {shape}")

# 打印模型输出信息
print("\nOutputs:")
for output_tensor in model.graph.output:
    print(f"Name: {output_tensor.name}")
    print(f"Type: {output_tensor.type.tensor_type.elem_type}")
    shape = [dim.dim_value for dim in output_tensor.type.tensor_type.shape.dim]
    print(f"Shape: {shape}")

model = onnxruntime.InferenceSession(model_path, providers=['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider'])

dummy_pts0 = np.random.rand(1, 100, 2).astype(np.float32)
dummy_desc0 = np.random.rand(1, 100, 256).astype(np.float32)
dummy_pts1 = np.random.rand(1, 102, 2).astype(np.float32)
dummy_desc1 = np.random.rand(1, 102, 256).astype(np.float32)

input0_name = model.get_inputs()[0].name
input1_name = model.get_inputs()[1].name
input2_name = model.get_inputs()[2].name
input3_name = model.get_inputs()[3].name

inputs = {input0_name: dummy_pts0, input1_name: dummy_pts1, input2_name: dummy_desc0, input3_name: dummy_desc1}

start_time = time()
output = model.run(None, inputs)
runtime = (time() - start_time) * 1000.
print("model dummy runtime is {}".format(runtime))
print("output's shape is ", np.squeeze(np.array(output), 0).shape)