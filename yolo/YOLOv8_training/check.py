import torch

print("CUDA 可用：", torch.cuda.is_available())
if torch.cuda.is_available():
    print("GPU 名稱：", torch.cuda.get_device_name(0))
else:
    print("無法偵測 GPU，仍使用 CPU。")
