#include <HandSolver/HandSolver.hpp>
#include <VisionProHandSolver/VisionProHandSolver.hpp>

HandSolver::HandSolver(){

}

HandSolver::~HandSolver(){

}

boost::shared_ptr<HandSolver> HandSolver::GetPtr(const HandSolver::BasicConfig &config_){
    switch (config_.type) {
        case XRBase::XRType::VisionPro :{
           return boost::make_shared<VisionProHandSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}


