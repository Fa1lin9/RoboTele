import numpy as np
from scipy.stats import linregress

def compute_linear_relations(x, y, z):
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)

    # Pearson 相关系数
    corr_xy = np.corrcoef(x, y)[0, 1]
    corr_xz = np.corrcoef(x, z)[0, 1]

    # 线性拟合
    reg_xy = linregress(x, y)
    reg_xz = linregress(x, z)

    # 拼接成 y = kx + b 的形式
    eq_xy = f"y = {reg_xy.slope:.6f} * x + {reg_xy.intercept:.6f}"
    eq_xz = f"z = {reg_xz.slope:.6f} * x + {reg_xz.intercept:.6f}"

    result = {
        "corr_xy": corr_xy,
        "corr_xz": corr_xz,
        "equation_xy": eq_xy,
        "equation_xz": eq_xz
    }
    return result


# 示例
x = [0, 1.410, 0.1, 0.244, 0.381, 0.556, 0.724, 0.861, 1.059, 1.265]
y = [0, 1.410, 0.1, 0.244, 0.381, 0.556, 0.724, 0.861, 1.059, 1.265]
z = [0, 1.62855, 0.1144, 0.2817, 0.4402, 0.6426, 0.8362, 0.994, 1.2236, 1.4613]

res = compute_linear_relations(x, y, z)
print(res)
