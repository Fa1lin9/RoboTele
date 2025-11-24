import csv
import numpy as np
import zmq
import struct
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from VisionProData import VisionProData_pb2

data = VisionProData_pb2.VisionProData()
# ==================== 配置 ====================

# 读取 CSV 文件
# file_name = "20250829_161326.csv"
# file_name = "20250929_152433.csv" # usually used
# file_name = "20251111_112715.csv"
# file_name = "20251114_105655.csv"
file_name = "20251117_163604.csv"
CSV_FILE = os.path.join(os.path.dirname(__file__), '..', 'data', f"{file_name}")

# 定义 ZeroMQ 配置
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://127.0.0.1:5555")
# socket = context.socket(zmq.PUSH)
# socket.bind("ipc:///tmp/teleoperate")

# 读取 CSV 文件中的数据
def load_data_from_csv(csv_file):
    poses = {'head_pose': [], 'left_arm_pose': [], 'right_arm_pose': []}

    # 读取 CSV 文件
    with open(csv_file, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # 跳过表头

        for row in reader:
            # 解析每列数据为 4x4 矩阵
            head_pose = np.array(eval(row[0]))  # 解析矩阵
            left_arm_pose = np.array(eval(row[1]))  # 解析矩阵
            right_arm_pose = np.array(eval(row[2]))  # 解析矩阵

            poses['head_pose'].append(head_pose)
            poses['left_arm_pose'].append(left_arm_pose)
            poses['right_arm_pose'].append(right_arm_pose)

    return poses

# 打包 4x4 矩阵为二进制
def pack_matrix(mat):
    """把 4x4 矩阵打包成二进制 double（8字节）"""
    flat = mat.flatten().astype(np.float64)  # double 8字节
    return struct.pack('d'*16, *flat)       # 'd' 对应 double

# 加载 CSV 数据
poses = load_data_from_csv(CSV_FILE)

# 发送数据的循环
count = 0
while count < len(poses['head_pose']):
    # 获取当前帧的姿态矩阵
    headPose = poses['head_pose'][count]
    leftArmPose = poses['left_arm_pose'][count]
    rightArmPose = poses['right_arm_pose'][count]

    # 打包连续发送 3 个矩阵
    data.headPose.data.extend(headPose.astype(float).flatten().tolist())
    data.leftArmPose.data.extend(leftArmPose.astype(float).flatten().tolist())
    data.rightArmPose.data.extend(rightArmPose.astype(float).flatten().tolist())

    msg = pack_matrix(headPose) + pack_matrix(leftArmPose) + pack_matrix(rightArmPose)
    print(f"Sent pose batch {count} with message size: {len(msg)} bytes")

    # 通过 ZeroMQ 发送数据
    socket.send(msg)
    count += 1

    # 控制发送间隔，模拟 50ms 的发送间隔
    # time.sleep(1 / 20)

print("Data sending complete!")
