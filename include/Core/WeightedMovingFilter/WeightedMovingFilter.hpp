#pragma once
#include <vector>
#include <deque>
#include <Eigen/Dense>
#include <numeric>

class WeightedMovingFilter
{
public:
    WeightedMovingFilter(const std::vector<double> weights_, size_t dataSize_);
    WeightedMovingFilter();
    ~WeightedMovingFilter();

    void Init(const std::vector<double> weights_, size_t dataSize_);

    void AddData(const Eigen::VectorXd& newData);

    Eigen::VectorXd GetFilteredData();

private:
    size_t windowSize;
    size_t dataSize;
    std::vector<double> weights;
    std::deque<Eigen::VectorXd> dataQueue;
    Eigen::VectorXd filteredData;

    Eigen::VectorXd ApplyFilter();
};
