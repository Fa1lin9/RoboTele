#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <sstream>
#include <ctime>
#include <Eigen/Dense>
#include <filesystem>

#include <source_path.h>

class CsvWriter {
public:
    // 构造函数1: 使用时间戳自动命名
    CsvWriter();

    // 构造函数2: 使用输入的文件名
    CsvWriter(const std::string& fileName);

    ~CsvWriter();

    void WriteEigenVector(const Eigen::VectorXd& vec);

    template<typename Derived>
    void WriteEigenMatrix(const Eigen::MatrixBase<Derived>& mat) {
        int count = 0;
        int total = mat.rows() * mat.cols();
        for (int i = 0; i < mat.rows(); ++i) {
            for (int j = 0; j < mat.cols(); ++j) {
                ofs_ << mat(i, j);
                if (++count != total) ofs_ << ",";
            }
        }
        ofs_ << "\n";
        ofs_.flush();
    }

private:
    std::ofstream ofs_;

    void OpenFile(const std::string& filename);

    std::string GetTimestamp();

    std::string GetDateStamp();

    std::string savePath;
};

