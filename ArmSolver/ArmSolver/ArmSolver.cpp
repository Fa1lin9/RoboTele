#include <ArmSolver/ArmSolver.hpp>
#include <Ti5DualArmSolver/Ti5DualArmSolver.hpp>

ArmSolver::ArmSolver(){

}

ArmSolver::~ArmSolver(){

}

boost::shared_ptr<ArmSolver> ArmSolver::GetPtr(const ArmSolver::BasicConfig &config_){
    switch (config_.type) {
        case RobotType::Ti5Robot :{
           return boost::make_shared<Ti5DualArmSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

const std::unordered_map<std::string, ArmSolver::Type> ArmSolver::typeMap = {
    {"Ti5DualArm", ArmSolver::Type::Ti5DualArm}
};

ArmSolver::Type ArmSolver::GetTypeFromStr(const std::string& str){
    auto temp = ArmSolver::typeMap.find(str);
    if(temp != ArmSolver::typeMap.end()){
        return temp->second;
    }

    throw std::invalid_argument("[RobotType::GetTypeFromStr] Invalid string");
}
