#include <ArmSolver/ArmSolver.hpp>
#include <Ti5DualArmSolver/Ti5DualArmSolver.hpp>
//#include <G1Dof23DualArmSolver/G1Dof23DualArmSolver.hpp>
#include <G1Dof29DualArmSolver/G1Dof29DualArmSolver.hpp>

ArmSolver::ArmSolver(){

}

ArmSolver::~ArmSolver(){

}

boost::shared_ptr<ArmSolver> ArmSolver::GetPtr(const ArmSolver::BasicConfig &config_){
    switch (config_.type) {
        case ArmSolver::Type::Ti5DualArm :{
           return boost::make_shared<Ti5DualArmSolver>(config_);
        }
//        case ArmSolver::Type::G1Dof23DualArm :{
//           return boost::make_shared<G1Dof23DualArm>(config_);
//        }
        case ArmSolver::Type::G1Dof29DualArm :{
           return boost::make_shared<G1Dof29DualArmSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

const std::unordered_map<std::string, ArmSolver::Type> ArmSolver::typeMap = {
    {"Ti5DualArm", ArmSolver::Type::Ti5DualArm},
    {"G1Dof23DualArm", ArmSolver::Type::G1Dof23DualArm},
    {"G1Dof29DualArm", ArmSolver::Type::G1Dof29DualArm},
};

ArmSolver::Type ArmSolver::GetTypeFromStr(const std::string& str){
    auto temp = ArmSolver::typeMap.find(str);
    if(temp != ArmSolver::typeMap.end()){
        return temp->second;
    }

    throw std::invalid_argument("[RobotType::GetTypeFromStr] Invalid string");
}
