import zmq
import struct
import numpy as np
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://127.0.0.1:5555")

def pack_matrix(mat):
    """把 4x4 矩阵打包成二进制 double（8字节）"""
    flat = mat.flatten().astype(np.float64)  # double 8字节
    return struct.pack('d'*16, *flat)       # 'd' 对应 double

count = 0
while True:
    # 随机示例矩阵
    headPose  = np.eye(4, dtype=np.float64) + count
    leftArmPose  = np.eye(4, dtype=np.float64) + count * 10
    rightArmPose = np.eye(4, dtype=np.float64) + count * 100

    # 打包连续发送 3 个矩阵
    msg = pack_matrix(headPose) + pack_matrix(leftArmPose) + pack_matrix(rightArmPose)
    print(len(msg))  # 每次 3 * 16 * 8 = 384 bytes
    socket.send(msg)
    print(f"Sent pose batch {count}")
    count += 1
    time.sleep(1/20)
