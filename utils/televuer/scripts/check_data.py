import csv
import numpy as np
import os

# ==================== 配置区（你只需要改这里） ====================

# CSV 文件名（位于 ../data/... 里）
file_name = "20251211_102209.csv"

# 打印前 N 个最大距离
TOP_N = 20

# 拼接 CSV 完整路径
CSV_FILE = os.path.join(os.path.dirname(__file__), '..', 'data', file_name)

# ==================== CSV 读取函数 ====================

def load_data_from_csv(csv_file):
    poses = {
        'head_pose': [],
        'left_arm_pose': [],
        'right_arm_pose': [],
    }

    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        headers = next(reader)

        idx_head = headers.index('Head Pose')
        idx_left_arm = headers.index('Left Arm Pose')
        idx_right_arm = headers.index('Right Arm Pose')

        for row in reader:
            poses['head_pose'].append(np.array(eval(row[idx_head])))
            poses['left_arm_pose'].append(np.array(eval(row[idx_left_arm])))
            poses['right_arm_pose'].append(np.array(eval(row[idx_right_arm])))

    return poses


# ==================== 欧式距离计算（矩阵全部元素） ====================

def matrix_distance(m1, m2):
    return np.linalg.norm(m1 - m2)


# ==================== 计算相邻帧距离（原功能） ====================

def compute_top_changes(matrix_list, top_n):
    distances = []
    for i in range(len(matrix_list) - 1):
        d = matrix_distance(matrix_list[i], matrix_list[i + 1])
        distances.append((i, i + 1, d))  # (前帧, 后帧, 距离)

    distances.sort(key=lambda x: x[2], reverse=True)
    return distances[:top_n]


# ==================== 计算同一帧内 Head vs Arm 的距离 ====================

def compute_frame_pair_distances(head_list, arm_list):
    """
    返回数组：
        [
            (frame_idx, distance),
            ...
        ]
    """
    results = []
    for i in range(len(head_list)):
        d = matrix_distance(head_list[i], arm_list[i])
        results.append((i, d))
    return results


def top_n_frame_pairs(dist_list, top_n):
    """
    输入格式为 [(idx, dist), ...]
    排序并返回前 N 个
    """
    dist_list = sorted(dist_list, key=lambda x: x[1], reverse=True)
    return dist_list[:top_n]


# ==================== ★ 新增功能：计算 Head vs Arm 的相邻帧矩阵距离变化 ====================

def compute_pair_temporal_changes(head_list, arm_list, top_n):
    """
    计算:
        dist_i   = distance( head[i], arm[i] )
        dist_i+1 = distance( head[i+1], arm[i+1] )
        delta    = abs( dist_i+1 - dist_i )

    返回：
        [
            (i, i+1, delta),
            ...
        ]
    """
    results = []
    for i in range(len(head_list) - 1):
        d1 = matrix_distance(head_list[i], arm_list[i])
        d2 = matrix_distance(head_list[i+1], arm_list[i+1])
        delta = abs(d2 - d1)

        results.append((i, i + 1, delta))

    results.sort(key=lambda x: x[2], reverse=True)
    return results[:top_n]


# ==================== 主程序 ====================

if __name__ == "__main__":
    print(f"Loading CSV: {CSV_FILE}")
    poses = load_data_from_csv(CSV_FILE)

    # ========== 第一部分：相邻帧变化（原功能） ==========
    print("\n==================== 相邻帧变化（Temporal Changes） ====================")

    print("\n===== Head Pose Matrix Changes =====")
    top_head = compute_top_changes(poses['head_pose'], TOP_N)
    for i, j, d in top_head:
        print(f"Frame {i} → {j}, distance = {d:.6f}")

    print("\n===== Left Arm Pose Matrix Changes =====")
    top_left = compute_top_changes(poses['left_arm_pose'], TOP_N)
    for i, j, d in top_left:
        print(f"Frame {i} → {j}, distance = {d:.6f}")

    print("\n===== Right Arm Pose Matrix Changes =====")
    top_right = compute_top_changes(poses['right_arm_pose'], TOP_N)
    for i, j, d in top_right:
        print(f"Frame {i} → {j}, distance = {d:.6f}")


    # ========== 第二部分：同一帧内 Head ↔ Arm 距离 ==========
    print("\n==================== 同一帧内矩阵距离（Head vs Arms） ====================")

    # ---- Head vs Left Arm ----
    head_left_dist = compute_frame_pair_distances(
        poses['head_pose'], poses['left_arm_pose'])

    print("\n===== Head ↔ Left Arm (same frame) =====")
    top_head_left = top_n_frame_pairs(head_left_dist, TOP_N)
    for idx, d in top_head_left:
        print(f"Frame {idx}, distance = {d:.6f}")

    # ---- Head vs Right Arm ----
    head_right_dist = compute_frame_pair_distances(
        poses['head_pose'], poses['right_arm_pose'])

    print("\n===== Head ↔ Right Arm (same frame) =====")
    top_head_right = top_n_frame_pairs(head_right_dist, TOP_N)
    for idx, d in top_head_right:
        print(f"Frame {idx}, distance = {d:.6f}")


    # ========== 第三部分（新增）：前后帧的 Head vs Arm 距离变化 ==========
    print("\n==================== Head vs Arm 前后帧距离变化（New Feature） ====================")

    # ---- Head ↔ Left Arm ----
    print("\n===== Head ↔ Left Arm：Temporal Delta =====")
    top_head_left_delta = compute_pair_temporal_changes(
        poses['head_pose'], poses['left_arm_pose'], TOP_N)

    for i, j, d in top_head_left_delta:
        print(f"Frame {i} → {j}, distance delta = {d:.6f}")

    # ---- Head ↔ Right Arm ----
    print("\n===== Head ↔ Right Arm：Temporal Delta =====")
    top_head_right_delta = compute_pair_temporal_changes(
        poses['head_pose'], poses['right_arm_pose'], TOP_N)

    for i, j, d in top_head_right_delta:
        print(f"Frame {i} → {j}, distance delta = {d:.6f}")

    print("\nAll computations finished.")
