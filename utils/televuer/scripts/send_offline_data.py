import csv
import numpy as np
import zmq
import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from VisionProData import VisionProData_pb2

# ==================== 配置 ====================
fps = 25
# new data
# file_name = "20251226_100815.csv" # sit down
# file_name = "20251226_112653.csv" # stand up

# new
file_name = "20260121_131848.csv"
# file_name = "20260121_134729.csv"

# for hand
# file_name = "20260126_152309.csv"
# file_name = "ok_gesture.csv"
# file_name = "thumbs_up_gesture.csv"
# file_name = "fist_gesture.csv"
CSV_FILE = os.path.join(os.path.dirname(__file__), '..', 'data', f"{file_name}")

# ZeroMQ
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://127.0.0.1:5555")


# ==================== 读取 CSV 文件（按你原始表头名字解析列） ====================
def load_data_from_csv(csv_file):
    poses = {
        'head_pose': [],
        'left_arm_pose': [],
        'right_arm_pose': [],
        'left_hand_positions': [],
        'right_hand_positions': [],
        'left_pinch_state': [],
        'left_pinch_value': [],
        'left_squeeze_state': [],
        'left_squeeze_value': [],
        'right_pinch_state': [],
        'right_pinch_value': [],
        'right_squeeze_state': [],
        'right_squeeze_value': [],
    }

    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        headers = next(reader)

        # 精确按你提供的表头名查列索引（严格匹配）
        idx_head = headers.index('Head Pose')
        idx_left_arm = headers.index('Left Arm Pose')
        idx_right_arm = headers.index('Right Arm Pose')
        idx_left_hand = headers.index('Left Hand (25x3)')
        idx_right_hand = headers.index('Right Hand (25x3)')
        idx_left_pinch_state = headers.index('Left PinchState')
        idx_left_pinch_value = headers.index('Left PinchValue')
        idx_left_squeeze_state = headers.index('Left SqueezeState')
        idx_left_squeeze_value = headers.index('Left SqueezeValue')
        idx_right_pinch_state = headers.index('Right PinchState')
        idx_right_pinch_value = headers.index('Right PinchValue')
        idx_right_squeeze_state = headers.index('Right SqueezeState')
        idx_right_squeeze_value = headers.index('Right SqueezeValue')

        for row in reader:
            poses['head_pose'].append(np.array(eval(row[idx_head])))
            poses['left_arm_pose'].append(np.array(eval(row[idx_left_arm])))
            poses['right_arm_pose'].append(np.array(eval(row[idx_right_arm])))
            poses['left_hand_positions'].append(np.array(eval(row[idx_left_hand])))
            poses['right_hand_positions'].append(np.array(eval(row[idx_right_hand])))

            poses['left_pinch_state'].append(eval(row[idx_left_pinch_state]))
            poses['left_pinch_value'].append(float(row[idx_left_pinch_value]))
            poses['left_squeeze_state'].append(eval(row[idx_left_squeeze_state]))
            poses['left_squeeze_value'].append(float(row[idx_left_squeeze_value]))

            poses['right_pinch_state'].append(eval(row[idx_right_pinch_state]))
            poses['right_pinch_value'].append(float(row[idx_right_pinch_value]))
            poses['right_squeeze_state'].append(eval(row[idx_right_squeeze_state]))
            poses['right_squeeze_value'].append(float(row[idx_right_squeeze_value]))

    return poses


# ==================== 发送循环 ====================
poses = load_data_from_csv(CSV_FILE)
count = 0

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
        p = data.leftHandPositions.joints.add()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)

    # ===== 右手点位 =====
    data.rightHandPositions.joints.clear()
    for x, y, z in rightHandPositions:
        p = data.rightHandPositions.joints.add()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)

    # ===== 左手手势 =====
    data.leftHandGesture.pinchState = bool(poses['left_pinch_state'][count])
    data.leftHandGesture.pinchValue = float(poses['left_pinch_value'][count])
    data.leftHandGesture.SqueezeState = bool(poses['left_squeeze_state'][count])
    data.leftHandGesture.SqueezeValue = float(poses['left_squeeze_value'][count])

    # ===== 右手手势 =====
    data.rightHandGesture.pinchState = bool(poses['right_pinch_state'][count])
    data.rightHandGesture.pinchValue = float(poses['right_pinch_value'][count])
    data.rightHandGesture.SqueezeState = bool(poses['right_squeeze_state'][count])
    data.rightHandGesture.SqueezeValue = float(poses['right_squeeze_value'][count])

    # ===== 发送 =====
    socket.send(data.SerializeToString())
    print(f"Sent pose batch {count}")

    count += 1
    time.sleep(1 / fps)

print("Data sending complete!")
