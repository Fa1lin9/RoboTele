#include <HandSolver/HandSolver.hpp>
#include <VisionProHandSolver/VisionProHandSolver.hpp>

HandSolver::HandSolver(){

}

HandSolver::~HandSolver(){

}

boost::shared_ptr<HandSolver> HandSolver::GetPtr(const HandSolver::BasicConfig &config_){
    switch (config_.type) {
        case HandSolver::Type::VisionPro :{
           return boost::make_shared<VisionProHandSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

const std::unordered_map<std::string, HandSolver::Type> HandSolver::typeMap = {
    {"VisionPro", HandSolver::Type::VisionPro}
};

HandSolver::Type HandSolver::GetTypeFromStr(const std::string& str){
    auto temp = HandSolver::typeMap.find(str);
    if(temp != HandSolver::typeMap.end()){
        return temp->second;
    }

    throw std::invalid_argument("[RobotType::GetTypeFromStr] Invalid string");
}

