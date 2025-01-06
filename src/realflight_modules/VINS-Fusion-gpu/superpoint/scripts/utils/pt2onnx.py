import torch
import torch.onnx
from GhostNet import SuperPointNet_GhostNet

weights_path = "../SuperPoint_GhostNet.pth.tar"
onnx_file_path = "sp_model.onnx"

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

model = SuperPointNet_GhostNet()
checkpoint = torch.load(weights_path)
model.load_state_dict(checkpoint['model_state_dict'])
model = model.to(device)
print('==> Load pre-trained network Successful.')
model.eval()

dummy_input = torch.randn(1, 1, 240, 320).to(device)
torch.onnx.export(model, dummy_input, onnx_file_path, verbose=True, input_names=['images'], output_names=['semi', 'coarse_desc'])