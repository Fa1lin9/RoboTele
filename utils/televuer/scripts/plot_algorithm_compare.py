import matplotlib.pyplot as plt
import numpy as np
import os

# ====================== 中文字体 ======================
from matplotlib import font_manager
font_path = "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc"
font_prop = font_manager.FontProperties(fname=font_path)
plt.rcParams['font.family'] = font_prop.get_name()
plt.rcParams['axes.unicode_minus'] = False

# ====================== 配置 ======================
SAVE_FLAG = True
FILE_PREFIX = "algo_compare_"

save_dir = os.path.join(os.path.dirname(__file__), '..', 'figs')
if SAVE_FLAG:
    os.makedirs(save_dir, exist_ok=True)

def get_save_path(filename):
    return os.path.join(save_dir, FILE_PREFIX + filename)


# ====================== 数据 ======================
methods = ["IPOPT", "SQP-qpOASES", "SQP-OSQP", "SQP-QRQP"]

# 位置误差（mm）
pos_labels = ["左臂X", "左臂Y", "左臂Z", "右臂X", "右臂Y", "右臂Z"]
pos_data = np.array([
    [3.99, 1.36, 2.76, 3.50, 1.35, 3.47],
    [22.33, 37.92, 18.98, 14.48, 20.51, 18.39],
    [21.37, 21.49, 17.23, 13.97, 18.24, 15.34],
    [26.76, 48.96, 32.91, 16.93, 25.46, 13.48]
])

# 姿态误差（rad）
rot_labels = ["左臂Roll", "左臂Pitch", "左臂Yaw",
              "右臂Roll", "右臂Pitch", "右臂Yaw"]
rot_data = np.array([
    [0.1262, 0.0620, 0.0635, 0.1591, 0.1145, 0.0708],
    [0.6288, 0.5239, 0.3282, 0.4222, 0.3149, 0.2336],
    [0.5342, 0.7337, 0.2805, 0.5592, 0.3984, 0.3234],
    [0.7314, 0.4858, 0.4373, 0.7878, 0.5470, 0.4467]
])


# ====================== 通用柱状图函数 ======================
def plot_grouped_bar(labels, data, ylabel, title, filename, value_format):

    plt.figure(figsize=(11, 5.5))

    n_methods = data.shape[0]
    n_labels = len(labels)

    x = np.arange(n_labels)
    width = 0.8 / n_methods

    max_val = np.max(data)
    offset = max_val * 0.015   # 标注偏移量

    for i in range(n_methods):
        bars = plt.bar(
            x + i * width,
            data[i],
            width=width,
            label=methods[i]
        )

        # 数值标注（防越界优化版）
        for j, v in enumerate(data[i]):
            y_text = min(v + offset, max_val * 1.12)

            plt.text(
                x[j] + i * width,
                y_text,
                value_format.format(v),
                ha='center',
                va='bottom',
                fontsize=9
            )

    # 👉 关键：增加上边界空间
    plt.ylim(0, max_val * 1.15)

    # 坐标轴与标题
    plt.xticks(
        x + width * (n_methods - 1) / 2,
        labels,
        rotation=30,
        fontsize=11
    )
    plt.ylabel(ylabel, fontsize=12)
    plt.title(title, fontsize=14)

    # 网格与图例
    plt.grid(axis='y', linestyle='--', alpha=0.5)
    plt.legend(fontsize=11)

    plt.tight_layout()
    plt.margins(x=0.02)

    if SAVE_FLAG:
        save_path = get_save_path(filename)
        plt.savefig(save_path, dpi=300)
        plt.close()
    else:
        plt.show()


# ====================== 绘制图1：位置误差 ======================
plot_grouped_bar(
    labels=pos_labels,
    data=pos_data,
    ylabel="平均绝对误差MAE(mm)",
    title="不同优化算法位置平均绝对误差对比",
    filename="position_error.png",
    value_format="{:.2f}"
)


# ====================== 绘制图2：姿态误差 ======================
plot_grouped_bar(
    labels=rot_labels,
    data=rot_data,
    ylabel="平均绝对误差MAE(rad)",
    title="不同优化算法姿态平均绝对误差对比",
    filename="rotation_error.png",
    value_format="{:.4f}"
)


# ====================== 完成提示 ======================
if SAVE_FLAG:
    print("算法对比图已全部保存！")