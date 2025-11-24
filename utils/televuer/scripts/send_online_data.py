import time
import numpy as np
from televuer import TeleVuer
from multiprocessing import shared_memory, Process, Array
import struct
from datetime import datetime
import os
import csv
from collections import deque

times = deque(maxlen=30)

# 获取当前时间戳并构建文件名
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILE = os.path.join(os.path.dirname(__file__), '..', 'data', f"{timestamp}.csv")

# 确保 'data' 目录存在
os.makedirs(os.path.dirname(CSV_FILE), exist_ok=True)

# ==================== 配置 ====================
frequency = 25
write_csv = True
IMG_SHAPE = (480, 640, 3)
IMG_SHM_NAME = "demo"
# IPC_PATH = "ipc:///tmp/tv_ipc"  # IPC socket 文件路径

# ==================== 共享内存 ====================
try:
    shm = shared_memory.SharedMemory(name=IMG_SHM_NAME)
except FileNotFoundError:
    shm = shared_memory.SharedMemory(
        create=True,
        size=np.prod(IMG_SHAPE),
        name=IMG_SHM_NAME
    )

img_array = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=shm.buf)
img_array[:] = 0

# ==================== TeleVuer 实例 ====================
tv = TeleVuer(
    binocular=False,
    use_hand_tracking=True,
    img_shape=IMG_SHAPE,
    img_shm_name=IMG_SHM_NAME,
    ngrok=False,
    webrtc=False
)

# ==================== IPC（ZeroMQ PUSH） ====================
# import socket
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sock.bind(('127.0.0.1', 5555))
# import redis
# r = redis.Redis(host='127.0.0.1', port=5555, db=0)

# ==================== 工具函数 ====================
def pack_matrix(mat):
    return struct.pack('d' * 16, *mat.flatten().astype(np.float64))

# ==================== 数据发送线程 ====================
def main(tv: TeleVuer):
    prev_head_pose = None
    prev_left_arm_pose = None
    prev_right_arm_pose = None
    count = 0

    import zmq
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://127.0.0.1:5555")

    with open(CSV_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Head Pose', 'Left Arm Pose', 'Right Arm Pose'])  # CSV表头

        while True:
            start = time.time()
            try:
                # 读取当前数据
                head_pose = tv.head_pose
                left_arm_pose = tv.left_arm_pose
                right_arm_pose = tv.right_arm_pose

                changed = False

                if prev_head_pose is None or not np.array_equal(prev_head_pose, head_pose):
                    print("Head Pose:\n", head_pose)
                    changed = True

                if prev_left_arm_pose is None or not np.array_equal(prev_left_arm_pose, left_arm_pose):
                    print("Left Arm Pose:\n", left_arm_pose)
                    changed = True

                if prev_right_arm_pose is None or not np.array_equal(prev_right_arm_pose, right_arm_pose):
                    print("Right Arm Pose:\n", right_arm_pose)
                    changed = True

                if changed:
                    msg = pack_matrix(head_pose) + pack_matrix(left_arm_pose) + pack_matrix(right_arm_pose)
                    socket.send(msg)
                    # sock.sendall(msg)


                    writer.writerow([head_pose.tolist(), left_arm_pose.tolist(), right_arm_pose.tolist()])
                    count += 1
                    print(f"[IPC] Sent batch {count}, size={len(msg)} bytes")

                # 更新上一帧
                prev_head_pose = head_pose.copy()
                prev_left_arm_pose = left_arm_pose.copy()
                prev_right_arm_pose = right_arm_pose.copy()

            except Exception as e:
                print("Error in IPC sender:", e)

            times.append(time.time() - start)
            fps = 1 / (sum(times) / len(times))
            if fps < 200:
                print(f"FPS: {fps:.2f}")

            end = time.time()
            time_elapsed = end - start
            sleep_time = max(0, (1 / float(frequency)) - time_elapsed)
            time.sleep(sleep_time)

        # time.sleep(0.01)  # 100Hz

if __name__ == '__main__':
    main(tv)