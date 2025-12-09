#include <WeightedMovingFilter/WeightedMovingFilter.hpp>

WeightedMovingFilter::WeightedMovingFilter(const std::vector<double> weights_, size_t dataSize_)
    :windowSize(weights_.size()),
     weights(weights_),
     dataSize(dataSize_),
     filteredData(Eigen::VectorXd::Zero(dataSize))
{
    double sum = std::accumulate(weights_.begin(), weights_.end(), 0.0);
    if(std::abs(sum - 1) > 1e-5){
        throw std::logic_error("The sum of the weights must be 1");
    }
}

WeightedMovingFilter::WeightedMovingFilter(){

}

WeightedMovingFilter::~WeightedMovingFilter(){

}

void WeightedMovingFilter::Init(const std::vector<double> weights_, size_t dataSize_)
{
    this->windowSize = weights_.size();
    this->weights = weights_;
    this->dataSize = dataSize_;
    this->filteredData = Eigen::VectorXd::Zero(this->dataSize);

    double sum = std::accumulate(weights_.begin(), weights_.end(), 0.0);
    if(std::abs(sum - 1) > 1e-5){
        throw std::logic_error("The sum of the weights must be 1");
    }
}

void WeightedMovingFilter::AddData(const Eigen::VectorXd &newData){
    if(static_cast<size_t>(newData.size()) != this->dataSize){
        throw std::logic_error("The new data's size must be equal to the given dataSize");
    }

    if(!dataQueue.empty()){
        if((dataQueue.back() - newData).norm() < 1e-5){
            return;
        }
    }

    if(dataQueue.size() >= windowSize){
        dataQueue.pop_front();
    }

    dataQueue.push_back(newData);

    this->filteredData = ApplyFilter();
}

Eigen::VectorXd WeightedMovingFilter::GetFilteredData(){
    return this->filteredData;
}

Eigen::VectorXd WeightedMovingFilter::ApplyFilter(){
    if(dataQueue.size() < windowSize){
        return dataQueue.back();
    }

    Eigen::MatrixXd dataMatrix(windowSize, dataSize);
    size_t row = 0;
    for(const auto& vec : dataQueue){
        dataMatrix.row(row++) = vec.transpose();
    }

    Eigen::VectorXd weights_(windowSize);
    for(size_t i=0;i<windowSize;i++){
        weights_(i) = this->weights[i];
    }

    Eigen::RowVectorXd filteredData = weights_.transpose() * dataMatrix;

    return filteredData.transpose();

}
