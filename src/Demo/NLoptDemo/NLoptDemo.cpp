#include <nlopt.hpp>
#include <iostream>
#include <vector>
#include <chrono>

// 目标函数 (Rosenbrock)，不使用梯度
double rosenbrock(const std::vector<double> &x, std::vector<double> &grad, void *data) {
    // 不填充 grad
    return (1 - x[0]) * (1 - x[0]) + 100.0 * (x[1] - x[0] * x[0]) * (x[1] - x[0] * x[0]);
}

#include <vector>
#include <cmath>

double complex_objective(const std::vector<double> &x, std::vector<double> &grad, void *data) {
    // 不填充 grad
    double term1 = std::sin(x[0]) * std::cos(x[1]);
    double term2 = std::exp(x[2]*x[2]);
    double term3 = (x[0] - x[1]) * (x[0] - x[1]);
    double term4 = std::log(1 + x[3]*x[3]);
    double term5 = x[0] * x[1] * x[2] * x[3];

    return term1 + term2 + term3 + term4 + term5;
}


int main() {
    try {
        // 使用无梯度算法 LN_NELDERMEAD
        nlopt::opt opt(nlopt::LN_NELDERMEAD, 2);  // 2维变量

        // 设置目标函数
        opt.set_min_objective(complex_objective, nullptr);

        // 设置收敛条件
        opt.set_ftol_rel(1e-9);
        opt.set_xtol_rel(1e-9);
        opt.set_maxeval(500);  // 最大迭代次数

        // 初始猜测
        std::vector<double> x = { -10.2, 100.0 };

        double minf; // 存储最优值
        auto start = std::chrono::high_resolution_clock::now();

        nlopt::result result = opt.optimize(x, minf);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "time consumed: " << duration.count() << " ms" << std::endl;

        std::cout << "优化结果: " << result << std::endl;
        std::cout << "最优解: x = " << x[0] << ", y = " << x[1] << std::endl;
        std::cout << "最优函数值: f(x,y) = " << minf << std::endl;
    }
    catch (std::exception &e) {
        std::cerr << "异常: " << e.what() << std::endl;
    }

    return 0;
}
