import requests
import numpy as np
import io
import cv2

# 定义服务端地址（例如运行在本地服务器上）
SERVER_URL = "http://10.184.17.105:8000/act"

# 定义要发送的图像和深度信息数组
image_array = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)  # 示例图像数组
depth_array = np.full((480, 640), 0.2).astype(np.float32)  # 示例深度数组，数据类型为 float32
proprio_array = np.random.rand(1,).astype(np.float32)
instr = "place cups"
timestep = 0

def send_data(image_array: np.ndarray, depth_array: np.ndarray, proprio_array):
    # 将图像数组转换为字节流
    _, image_encoded = cv2.imencode('.jpg', image_array)
    image_bytes = io.BytesIO(image_encoded.tobytes())

    # 将深度数组转换为字节流（使用 .npy 格式）
    depth_bytes = io.BytesIO()
    np.save(depth_bytes, depth_array)
    depth_bytes.seek(0)
    
    proprio_bytes = io.BytesIO()
    np.save(proprio_bytes, proprio_array)
    proprio_bytes.seek(0)

    # 定义要发送的数据
    files = {
        "image_file": ("image.jpg", image_bytes, "image/jpeg"),
        "depth_file": ("depth.npy", depth_bytes, "application/octet-stream"),
        "proprio_file": ("proprio.npy", proprio_bytes, "application/octet-stream"),
    }
    
    data = {
        "instr": instr,
        "timestep": timestep
    }

    # 发送 POST 请求到服务端
    response = requests.post(SERVER_URL, files=files, data=data)

    # 打印服务端返回的结果
    if response.status_code == 200:
        print("Server response:", response.json())
    else:
        print("Failed to get a valid response from server. Status code:", response.status_code)

if __name__ == "__main__":
    # 调用函数发送数据到服务端
    send_data(image_array, depth_array)