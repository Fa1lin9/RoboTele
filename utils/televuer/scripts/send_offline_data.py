import csv
import numpy as np
import zmq
import struct
import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from VisionProData import VisionProData_pb2

# ==================== 配置 ====================

# 读取 CSV 文件
# file_name = "20250829_161326.csv"
# file_name = "20250929_152433.csv" # usually used
# file_name = "20251117_163604.csv"
file_name = "20251127_105921.csv"
CSV_FILE = os.path.join(os.path.dirname(__file__), '..', 'data', f"{file_name}")

# 定义 ZeroMQ 配置
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://127.0.0.1:5555")
# socket = context.socket(zmq.PUSH)
# socket.bind("ipc:///tmp/teleoperate")

# ==================== 读取 CSV 文件 ====================
def load_data_from_csv(csv_file):
    """
    返回字典：
    poses = {
        'head_pose': [np.array(4x4), ...],
        'left_arm_pose': [...],
        'right_arm_pose': [...],
        'left_hand_positions': [np.array(25x3), ...],
        'right_hand_positions': [np.array(25x3), ...]
    }
    """
    poses = {
        'head_pose': [],
        'left_arm_pose': [],
        'right_arm_pose': [],
        'left_hand_positions': [],
        'right_hand_positions': []
    }

    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        headers = next(reader)  # 跳过表头

        for row in reader:
            head_pose = np.array(eval(row[0]))
            left_arm_pose = np.array(eval(row[1]))
            right_arm_pose = np.array(eval(row[2]))
            left_hand_positions = np.array(eval(row[3]))
            right_hand_positions = np.array(eval(row[4]))

            poses['head_pose'].append(head_pose)
            poses['left_arm_pose'].append(left_arm_pose)
            poses['right_arm_pose'].append(right_arm_pose)
            poses['left_hand_positions'].append(left_hand_positions)
            poses['right_hand_positions'].append(right_hand_positions)

    return poses


# ==================== 发送循环 ====================
poses = load_data_from_csv(CSV_FILE)
count = 0
fps = 25

while count < len(poses['head_pose']):
    data = VisionProData_pb2.VisionProData()

    # 当前帧
    headPose = poses['head_pose'][count]
    leftArmPose = poses['left_arm_pose'][count]
    rightArmPose = poses['right_arm_pose'][count]
    leftHandPositions = poses['left_hand_positions'][count]
    rightHandPositions = poses['right_hand_positions'][count]

    # ===== 3 个矩阵 =====
    data.headPose.data.extend(headPose.astype(float).flatten().tolist())
    data.leftArmPose.data.extend(leftArmPose.astype(float).flatten().tolist())
    data.rightArmPose.data.extend(rightArmPose.astype(float).flatten().tolist())

    # ===== 左手点位 =====
    data.leftHandPositions.joints.clear()
    for x, y, z in leftHandPositions:
        joint = data.leftHandPositions.joints.add()
        joint.x = float(x)
        joint.y = float(y)
        joint.z = float(z)

    # ===== 右手点位 =====
    data.rightHandPositions.joints.clear()
    for x, y, z in rightHandPositions:
        joint = data.rightHandPositions.joints.add()
        joint.x = float(x)
        joint.y = float(y)
        joint.z = float(z)

    # 发送
    socket.send(data.SerializeToString())
    print(f"Sent pose batch {count}")

    count += 1
    time.sleep(1 / fps)

print("Data sending complete!")
