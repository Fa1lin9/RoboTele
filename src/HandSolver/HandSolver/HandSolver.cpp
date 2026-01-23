#include <HandSolver/HandSolver.hpp>
#include <VisionProHandSolver/VisionProHandSolver.hpp>

HandSolver::HandSolver(){

}

HandSolver::~HandSolver(){

}

std::vector<double> HandSolver::GetLowerBound() {
    std::cout << "[HandSolver::GetLowerBound] called" << std::endl;
    return {};
}

std::vector<double> HandSolver::GetUpperBound() {
    std::cout << "[HandSolver::GetUpperBound] called" << std::endl;
    return {};
}

std::vector<std::string> HandSolver::GetFingersName() {
    std::cout << "[HandSolver::GetFingersName] called" << std::endl;
    return {};
}

std::shared_ptr<HandSolver> HandSolver::GetPtr(const HandSolver::BasicConfig &config_){
    switch (config_.type) {
        case XRBase::XRType::VisionPro :{
           return std::make_shared<VisionProHandSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}


