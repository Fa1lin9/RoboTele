import time
import numpy as np
from televuer import TeleVuer
from multiprocessing import shared_memory
from datetime import datetime
import os
import csv
from collections import deque
import sys
import traceback

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from VisionProData import VisionProData_pb2

# ==================== 配置 ====================
FREQUENCY = 25
IMG_SHAPE = (480, 640, 3)
IMG_SHM_NAME = "demo"

# ==================== CSV ====================
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILE = os.path.join(os.path.dirname(__file__), '..', 'data', f"{timestamp}.csv")
os.makedirs(os.path.dirname(CSV_FILE), exist_ok=True)

# ==================== 共享内存 ====================
try:
    shm = shared_memory.SharedMemory(name=IMG_SHM_NAME)
except FileNotFoundError:
    shm = shared_memory.SharedMemory(create=True, size=np.prod(IMG_SHAPE), name=IMG_SHM_NAME)

img_array = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=shm.buf)
img_array[:] = 0

# ==================== ZeroMQ ====================
import zmq

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://127.0.0.1:5555")

# ==================== TeleVuer ====================
tv = TeleVuer(
    binocular=False,
    use_hand_tracking=True,
    img_shape=IMG_SHAPE,
    img_shm_name=IMG_SHM_NAME,
    ngrok=False,
    webrtc=False
)

# ==================== 主循环 ====================
def main(tv):
    prev_head = prev_left = prev_right = None

    # CSV 初始化（加入左右手）
    csv_file = open(CSV_FILE, 'w', newline='')
    writer = csv.writer(csv_file)
    writer.writerow([
        'Head Pose',
        'Left Arm Pose',
        'Right Arm Pose',
        'Left Hand (25x3)',
        'Right Hand (25x3)'
    ])

    frame_times = deque(maxlen=30)

    while True:
        frame_start = time.time()

        try:
            # ===== 获取数据 =====
            head = tv.head_pose
            left = tv.left_arm_pose
            right = tv.right_arm_pose
            left_hand = tv.left_hand_positions
            right_hand = tv.right_hand_positions

            # ===== 基础异常检查 =====
            if head is None or left is None or right is None:
                print("[WARN] Pose is None, skipping frame.")
                time.sleep(0.01)
                continue

            if head.shape != (4, 4) or left.shape != (4, 4) or right.shape != (4, 4):
                print("[WARN] Invalid matrix shape, skipping.")
                continue

            if left_hand is None or right_hand is None:
                print("[WARN] Hand positions missing.")
                left_hand = np.zeros((25, 3))
                right_hand = np.zeros((25, 3))

            if left_hand.shape != (25, 3) or right_hand.shape != (25, 3):
                print("[WARN] Invalid hand shape, restoring zeros.")
                left_hand = np.zeros((25, 3))
                right_hand = np.zeros((25, 3))

            # ===== 检查是否变化 =====
            changed = (
                prev_head is None or
                not np.array_equal(prev_head, head) or
                not np.array_equal(prev_left, left) or
                not np.array_equal(prev_right, right)
            )

            if changed:
                data = VisionProData_pb2.VisionProData()

                # ===== 矩阵填充（修复你原本的错误用法）=====
                data.headPose.data.clear()
                data.headPose.data.extend(head.flatten().astype(float))

                data.leftArmPose.data.clear()
                data.leftArmPose.data.extend(left.flatten().astype(float))

                data.rightArmPose.data.clear()
                data.rightArmPose.data.extend(right.flatten().astype(float))

                # ===== 左手 =====
                data.leftHandPositions.joints.clear()
                for x, y, z in left_hand:
                    p = data.leftHandPositions.joints.add()
                    p.x = float(x)
                    p.y = float(y)
                    p.z = float(z)

                # ===== 右手 =====
                data.rightHandPositions.joints.clear()
                for x, y, z in right_hand:
                    p = data.rightHandPositions.joints.add()
                    p.x = float(x)
                    p.y = float(y)
                    p.z = float(z)

                # ===== 发送 =====
                socket.send(data.SerializeToString())

                # ===== 写 CSV =====
                writer.writerow([
                    head.tolist(),
                    left.tolist(),
                    right.tolist(),
                    left_hand.tolist(),
                    right_hand.tolist()
                ])

                print(">>> New pose updated & sent.")

            # 更新上一帧
            prev_head = head.copy()
            prev_left = left.copy()
            prev_right = right.copy()

        except Exception as e:
            print("[ERROR] Exception in main loop:", e)
            traceback.print_exc()

        # FPS
        frame_times.append(time.time() - frame_start)
        fps = 1 / np.mean(frame_times)
        if fps < 60:
            print(f"FPS: {fps:.2f}")

        # 控制频率
        time.sleep(max(0, 1 / FREQUENCY - (time.time() - frame_start)))

    csv_file.close()


if __name__ == '__main__':
    main(tv)
