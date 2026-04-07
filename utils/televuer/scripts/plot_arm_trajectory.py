import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

# ====================== 中文字体 ======================
from matplotlib import font_manager
font_path = "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc"
font_prop = font_manager.FontProperties(fname=font_path)
plt.rcParams['font.family'] = font_prop.get_name()
plt.rcParams['axes.unicode_minus'] = False

# ====================== 配置 ======================

# 调参完成的： CrpRobot H1 Ti5Robot G1Dof29 GR1 X1 N1

SAVE_FLAG = True
FILE_PREFIX = "(X1)"  # 文件名前缀
file_name = "20260407_170805.csv"
CSV_FILE = os.path.join(os.path.dirname(__file__), '..', 'data', file_name)

# ====================== 读取 CSV ======================
df = pd.read_csv(CSV_FILE, header=None)

# 拆分数据
Lr_pos = df.iloc[:, 0:3]; Lh_pos = df.iloc[:, 3:6]
Rr_pos = df.iloc[:, 6:9]; Rh_pos = df.iloc[:, 9:12]

Lr_euler = df.iloc[:, 12:15]; Lh_euler = df.iloc[:, 15:18]
Rr_euler = df.iloc[:, 18:21]; Rh_euler = df.iloc[:, 21:24]

solve_time = df.iloc[:, 24]

# 设置列名
for df_part in [Lr_pos, Lh_pos, Rr_pos, Rh_pos]:
    df_part.columns = ["X", "Y", "Z"]
for df_part in [Lr_euler, Lh_euler, Rr_euler, Rh_euler]:
    df_part.columns = ["Roll", "Pitch", "Yaw"]

t = range(len(df))

# 创建保存文件夹
save_dir = os.path.join(os.path.dirname(__file__), '..', 'figs')
if SAVE_FLAG:
    os.makedirs(save_dir, exist_ok=True)

# ====================== 统一获取保存路径函数 ======================
def get_save_path(filename):
    return os.path.join(save_dir, FILE_PREFIX + filename)

# ====================== 绘制曲线 ======================
def plot_arm_axis(arm, axis, human_df, robot_df, mode="Pos"):
    plt.figure(figsize=(6,4))
    plt.plot(t, human_df[axis], label="人体参考轨迹", linewidth=1.5)
    plt.plot(t, robot_df[axis], label="机器人执行轨迹", linewidth=1.5)
    plt.xlabel("时间步", fontsize=12)
    ylabel = "位置 (m)" if mode == "Pos" else "欧拉角 (rad)"
    plt.ylabel(ylabel, fontsize=12)
    if mode == "Pos":
        plt.title(f"{arm}臂{axis}轴方向位置变化", fontsize=14)
    else:
        plt.title(f"{arm}臂{axis}角方向欧拉角变化", fontsize=14)
    plt.legend(fontsize=11)
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    if SAVE_FLAG:
        # 暂时不绘制单独的
        # save_path = get_save_path(f"{arm}_{axis}_{mode}.png")
        # plt.savefig(save_path, dpi=300)
        print("暂时不绘制单独的")
    else:
        plt.show()

# ====================== 绘制求解时间曲线 ======================
def plot_solve_time_curve():
    mean_time = solve_time.mean()
    std_time = solve_time.std()
    plt.figure(figsize=(6,4))
    plt.plot(t, solve_time, label="求解时间", color="purple", alpha=0.7)
    plt.axhline(mean_time, color='red', linestyle='--', label=f'平均值 = {mean_time:.2f} ms')
    plt.fill_between(t, mean_time - std_time, mean_time + std_time, color='red', alpha=0.2,
                     label=f'标准差 = {std_time:.2f} ms')
    plt.xlabel("时间步", fontsize=12)
    plt.ylabel("求解时间 (ms)", fontsize=12)
    plt.title("每步优化求解时间", fontsize=14)
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend(fontsize=11)
    plt.tight_layout()
    if SAVE_FLAG:
        save_path = get_save_path("solve_time.png")
        plt.savefig(save_path, dpi=300)
        plt.close()
    else:
        plt.show()

# ====================== 计算平均误差 ======================
def compute_mean_axis_error(human_df, robot_df):
    return (robot_df - human_df).abs().mean()

# ====================== 绘图流程 ======================
pos_labels, pos_errors = [], []
euler_labels, euler_errors = [], []

for arm_name, human_pos, robot_pos, human_euler, robot_euler in [
    ("左", Lh_pos, Lr_pos, Lh_euler, Lr_euler),
    ("右", Rh_pos, Rr_pos, Rh_euler, Rr_euler)
]:
    # 平移曲线
    for axis in ["X", "Y", "Z"]:
        plot_arm_axis(arm_name, axis, human_pos, robot_pos, mode="Pos")
    # 欧拉角曲线
    for axis in ["Roll", "Pitch", "Yaw"]:
        plot_arm_axis(arm_name, axis, human_euler, robot_euler, mode="Euler")

    # 平均误差
    mean_errors_pos = compute_mean_axis_error(human_pos, robot_pos)
    mean_errors_euler = compute_mean_axis_error(human_euler, robot_euler)

    for axis in ["X", "Y", "Z"]:
        pos_labels.append(f"{arm_name}臂{axis}轴")
        pos_errors.append(mean_errors_pos[axis])
    for axis in ["Roll", "Pitch", "Yaw"]:
        euler_labels.append(f"{arm_name}臂{axis}角")
        euler_errors.append(mean_errors_euler[axis])

# ====================== 绘制平移误差柱状图（改为毫米） ======================
plt.figure(figsize=(12, 5))
pos_errors_mm = [v*1000 for v in pos_errors]
bars = plt.bar(pos_labels, pos_errors_mm, width=0.4)
max_val = max(pos_errors_mm)
plt.ylim(0, max_val*1.2)
plt.ylabel("平均绝对误差MAE(mm)", fontsize=12)
plt.title("左右臂各轴向位置平均绝对误差", fontsize=14)
plt.grid(axis='y', linestyle='--', alpha=0.5)
plt.xticks(rotation=30, fontsize=11)
for i, v in enumerate(pos_errors_mm):
    plt.text(i, v + max_val*0.02, f"{v:.2f}", ha='center', va='bottom', fontsize=10)
plt.tight_layout()
if SAVE_FLAG:
    save_path = get_save_path("pos_axis_errors_mm.png")
    plt.savefig(save_path, dpi=300)
    plt.close()

# ====================== 绘制欧拉角误差柱状图 ======================
plt.figure(figsize=(12, 5))
bars = plt.bar(euler_labels, euler_errors, width=0.4)
max_val = max(euler_errors)
plt.ylim(0, max_val*1.2)
plt.ylabel("平均绝对误差MAE(rad)", fontsize=12)
plt.title("左右臂各轴向欧拉角平均绝对误差", fontsize=14)
plt.grid(axis='y', linestyle='--', alpha=0.5)
plt.xticks(rotation=30, fontsize=11)
for i, v in enumerate(euler_errors):
    plt.text(i, v + max_val*0.02, f"{v:.4f}", ha='center', va='bottom', fontsize=10)
plt.tight_layout()
if SAVE_FLAG:
    save_path = get_save_path("euler_axis_errors.png")
    plt.savefig(save_path, dpi=300)
    plt.close()

# ====================== 求解时间 ======================
plot_solve_time_curve()

# ====================== 新增：XYZ 横向拼接图（保持原标题风格） ======================
def plot_xyz_combined(arm_name, human_pos, robot_pos):
    fig, axes = plt.subplots(1, 3, figsize=(15,4))
    for i, axis in enumerate(["X","Y","Z"]):
        axes[i].plot(t, human_pos[axis], label="人体参考轨迹", linewidth=1.5)
        axes[i].plot(t, robot_pos[axis], label="机器人执行轨迹", linewidth=1.5)
        axes[i].set_xlabel("时间步", fontsize=11)
        axes[i].set_ylabel("位置 (m)", fontsize=11)
        axes[i].set_title(f"{arm_name}臂{axis}轴方向位置变化", fontsize=12)  # 保持原来标题风格
        axes[i].grid(True, linestyle='--', alpha=0.5)
        if i == 0:
            axes[i].legend(fontsize=10)
    plt.tight_layout()
    if SAVE_FLAG:
        save_path = get_save_path(f"{arm_name}_XYZ_combined.png")
        plt.savefig(save_path, dpi=300)
        plt.close()
    else:
        plt.show()

# ====================== 新增：RPY 横向拼接图（保持原标题风格） ======================
def plot_rpy_combined(arm_name, human_euler, robot_euler):
    fig, axes = plt.subplots(1, 3, figsize=(15,4))
    for i, axis in enumerate(["Roll","Pitch","Yaw"]):
        axes[i].plot(t, human_euler[axis], label="人体参考轨迹", linewidth=1.5)
        axes[i].plot(t, robot_euler[axis], label="机器人执行轨迹", linewidth=1.5)
        axes[i].set_xlabel("时间步", fontsize=11)
        axes[i].set_ylabel("欧拉角 (rad)", fontsize=11)
        axes[i].set_title(f"{arm_name}臂{axis}角方向欧拉角变化", fontsize=12)  # 保持原来标题风格
        axes[i].grid(True, linestyle='--', alpha=0.5)
        if i == 0:
            axes[i].legend(fontsize=10)
    plt.tight_layout()
    if SAVE_FLAG:
        save_path = get_save_path(f"{arm_name}_RPY_combined.png")
        plt.savefig(save_path, dpi=300)
        plt.close()
    else:
        plt.show()

# ====================== 绘制拼接图 ======================
for arm_name, human_pos, robot_pos, human_euler, robot_euler in [
    ("左", Lh_pos, Lr_pos, Lh_euler, Lr_euler),
    ("右", Rh_pos, Rr_pos, Rh_euler, Rr_euler)
]:
    plot_xyz_combined(arm_name, human_pos, robot_pos)
    plot_rpy_combined(arm_name, human_euler, robot_euler)

if SAVE_FLAG:
    print("所有图形已保存！")

# ====================== 新增：XYZ + RPY 拼接大图 ======================
def plot_xyz_rpy_combined(arm_name, human_pos, robot_pos, human_euler, robot_euler):
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))

    # 调整子图间距（增强分块感）
    plt.subplots_adjust(wspace=0.3, hspace=0.35)

    # 上行：XYZ
    for i, axis in enumerate(["X", "Y", "Z"]):
        axes[0, i].plot(t, human_pos[axis], label="人体参考轨迹", linewidth=1.5)
        axes[0, i].plot(t, robot_pos[axis], label="机器人执行轨迹", linewidth=1.5)
        axes[0, i].set_xlabel("时间步", fontsize=11)
        axes[0, i].set_ylabel("位置 (m)", fontsize=11)
        axes[0, i].set_title(f"{arm_name}臂{axis}轴方向位置变化", fontsize=12)
        axes[0, i].grid(True, linestyle='--', alpha=0.5)

        # 👉 加粗边框（关键）
        for spine in axes[0, i].spines.values():
            spine.set_linewidth(1.2)

        if i == 0:
            axes[0, i].legend(fontsize=10)

    # 下行：RPY
    for i, axis in enumerate(["Roll", "Pitch", "Yaw"]):
        axes[1, i].plot(t, human_euler[axis], label="人体参考轨迹", linewidth=1.5)
        axes[1, i].plot(t, robot_euler[axis], label="机器人执行轨迹", linewidth=1.5)
        axes[1, i].set_xlabel("时间步", fontsize=11)
        axes[1, i].set_ylabel("欧拉角 (rad)", fontsize=11)
        axes[1, i].set_title(f"{arm_name}臂{axis}角方向欧拉角变化", fontsize=12)
        axes[1, i].grid(True, linestyle='--', alpha=0.5)

        # 👉 加粗边框
        for spine in axes[1, i].spines.values():
            spine.set_linewidth(1.2)

        if i == 0:
            axes[1, i].legend(fontsize=10)

    # 👉 可选：加一条横向分界线（区分XYZ和RPY）
    fig.add_artist(plt.Line2D([0.0, 1.0], [0.5, 0.5],
                             transform=fig.transFigure,
                             color='black', linewidth=1.2))

    plt.tight_layout()

    if SAVE_FLAG:
        save_path = get_save_path(f"{arm_name}_XYZ_RPY_combined.png")
        plt.savefig(save_path, dpi=300)
        plt.close()
    else:
        plt.show()


# ====================== 绘制大拼接图 ======================
for arm_name, human_pos, robot_pos, human_euler, robot_euler in [
    ("左", Lh_pos, Lr_pos, Lh_euler, Lr_euler),
    ("右", Rh_pos, Rr_pos, Rh_euler, Rr_euler)
]:
    plot_xyz_rpy_combined(arm_name, human_pos, robot_pos, human_euler, robot_euler)