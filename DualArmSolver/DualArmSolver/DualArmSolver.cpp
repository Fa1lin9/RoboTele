#include <DualArmSolver/DualArmSolver.hpp>
#include <Ti5RobotDualArmSolver/Ti5RobotDualArmSolver.hpp>

DualArmSolver::DualArmSolver(){

}

DualArmSolver::~DualArmSolver(){

}

boost::shared_ptr<DualArmSolver> DualArmSolver::GetPtr(const DualArmSolver::BasicConfig &config_){
    switch (config_.robotType) {
        case RobotType::Ti5Robot :{
           return boost::make_shared<Ti5RobotDualArmSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}



