import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
#from pycuda.tools import make_context_current

class SuperPointNet_TensorRT:
  def __init__(self, engine_path):
    self.engine_path = engine_path

  def engine_init(self):
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    with open(self.engine_path, 'rb') as f, trt.Runtime(TRT_LOGGER) as runtime:
      self.engine = runtime.deserialize_cuda_engine(f.read())
      # inspector = self.engine.create_engine_inspector()
      # print('engine layer_info:\n{}'.format(inspector.get_engine_information(trt.LayerInformationFormat(1))))
    self.inputs = [] # [(host_input, device_input)]
    self.outputs = [] # [(host_output, device_output)]
    self.tensors = [] # [device_input, device_output]
    self.stream = cuda.Stream()
    for i in range(self.engine.num_io_tensors):
      name = self.engine.get_tensor_name(i)
      if i == 0:
        self.input_shape = self.engine.get_tensor_shape(name)
      elif i == 1:
        self.output_semi_shape = self.engine.get_tensor_shape(name)
      elif i == 2:
        self.output_desc_shape = self.engine.get_tensor_shape(name)
    # allocate memory
    self.allocate_buffers()
    self.context = self.engine.create_execution_context()# Create context once

  def allocate_buffers(self):
    self.inputs.clear()
    self.outputs.clear()
    self.tensors.clear()
    for tensor in self.engine:
      size = trt.volume(self.engine.get_tensor_shape(tensor))
      dtype = trt.nptype(self.engine.get_tensor_dtype(tensor))
      # allocate device memory
      buffer_device = cuda.mem_alloc(size * np.dtype(dtype).itemsize)
      self.tensors.append(int(buffer_device))

      if self.engine.get_tensor_mode(tensor) == trt.TensorIOMode.INPUT:
        # Allocate page-locked memory for input
        input_host = cuda.pagelocked_empty(size, dtype)
        self.inputs.append((input_host, buffer_device))
      else:
        # Allocate page-locked memory for output
        output_host = cuda.pagelocked_empty(size, dtype)
        self.outputs.append((output_host, buffer_device))
  
  def infer(self, input_image):
    if input_image.ndim == 2:  # 输入为 (240, 320)
        input_image = np.expand_dims(input_image, axis=0)  # 添加通道维度，(1, 240, 320)
        input_image = np.expand_dims(input_image, axis=0)  # 添加批次维度，(1, 1, 240, 320)
    elif input_image.ndim == 3 and input_image.shape[0] == 1:  # 输入为 (1, 240, 320)
        input_image = np.expand_dims(input_image, axis=0)  # 添加批次维度，(1, 1, 240, 320)
    else:
        assert input_image.shape == tuple(self.input_shape), (
            f"Input shape mismatch. Expected {self.input_shape}, got {input_image.shape}"
        )
    
    input_host, input_device = self.inputs[0]
    output_host1, output_device1 = self.outputs[0]
    output_host2, output_device2 = self.outputs[1]
    # Copy input data to page-locked memory
    input_image_contiguous = np.ascontiguousarray(input_image)
    np.copyto(input_host, input_image_contiguous.ravel())
    # Copy input to device
    cuda.memcpy_htod_async(input_device, input_host, self.stream)
    # Execute inference
    try:
      self.context.execute_async_v2(bindings=self.tensors, stream_handle=self.stream.handle)
    except Exception as e:
      print(f"Inference failed: {e}")
    # Copy output from device to host
    cuda.memcpy_dtoh_async(output_host1, output_device1, self.stream)
    cuda.memcpy_dtoh_async(output_host2, output_device2, self.stream)
    self.stream.synchronize()

    # Reshape outputs
    output1 = np.reshape(output_host1, self.output_semi_shape)
    output2 = np.reshape(output_host2, self.output_desc_shape)
    return output1, output2