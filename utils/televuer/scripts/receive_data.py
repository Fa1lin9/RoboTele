import zmq
import numpy as np
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from VisionProData import VisionProData_pb2


def recv_loop():
    # ZeroMQ SUB 端
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://127.0.0.1:5555")
    socket.setsockopt(zmq.SUBSCRIBE, b"")

    print("Receiver started, waiting for messages...")

    while True:
        try:
            data_bytes = socket.recv()
            msg = VisionProData_pb2.VisionProData()
            msg.ParseFromString(data_bytes)

            # 还原 4x4 矩阵（列主序 or 行主序取决于发送端，这里按你的发送方式：flatten 行主序）
            headPose = np.array(msg.headPose.data, dtype=float).reshape((4, 4))
            leftArmPose = np.array(msg.leftArmPose.data, dtype=float).reshape((4, 4))
            rightArmPose = np.array(msg.rightArmPose.data, dtype=float).reshape((4, 4))

            print("\n=== Received Frame ===")
            print("Head Pose:")
            print(headPose)
            print("Left Arm Pose:")
            print(leftArmPose)
            print("Right Arm Pose:")
            print(rightArmPose)

        except KeyboardInterrupt:
            print("Receiver stopped.")
            break


if __name__ == "__main__":
    recv_loop()
