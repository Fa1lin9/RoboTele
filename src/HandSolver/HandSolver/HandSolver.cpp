#include <HandSolver/HandSolver.hpp>
#include <VisionProHandSolver/VisionProHandSolver.hpp>

HandSolver::HandSolver(){

}

HandSolver::~HandSolver(){

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


