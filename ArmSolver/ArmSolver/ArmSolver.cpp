#include <ArmSolver/ArmSolver.hpp>
#include <Ti5DualArmSolver/Ti5DualArmSolver.hpp>
#include <G1Dof23DualArmSolver/G1Dof23DualArmSolver.hpp>
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
        case ArmSolver::Type::G1Dof23DualArm :{
           return boost::make_shared<G1Dof23DualArmSolver>(config_);
        }
        case ArmSolver::Type::G1Dof29DualArm :{
           return boost::make_shared<G1Dof29DualArmSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

boost::shared_ptr<ArmSolver> ArmSolver::GetPtr(const std::string& filePath){
    JsonParser jsonParser(filePath);
    json::object rootObj = jsonParser.GetJsonObject();

    // ArmSolver
    ArmSolver::BasicConfig config = {
        .type = ArmSolver::GetTypeFromStr(rootObj["Type"].as_string().c_str()),
        .baseFrameName = JsonParser::JsonArray2StdVecStr(rootObj["BaseFrameName"].as_array()),
        .targetFrameName = JsonParser::JsonArray2StdVecStr(rootObj["TargetFrameName"].as_array()),
        .maxIteration = static_cast<int>(rootObj["MaxIteration"].as_int64()),
        .relativeTol = rootObj["RelativeTol"].as_double(),
        .dofArm = JsonParser::JsonArray2StdVecInt(rootObj["DofArm"].as_array()),
        .wTranslation = rootObj["ObjectiveFunc"].as_object()["TranslationWeight"].as_double(),
        .wRotation = rootObj["ObjectiveFunc"].as_object()["RotationWeight"].as_double(),
        .wRegularization = rootObj["ObjectiveFunc"].as_object()["RegularizationWeight"].as_double(),
        .wSmooth = rootObj["ObjectiveFunc"].as_object()["SmoothWeight"].as_double(),
    };
    // In the future, the variable BaseOffset maybe not just 1
    // So I choose to set BaseOffset to 3-D array
    // For BaseOffset
    std::vector<Eigen::Matrix4d> baseOffset;
    for(size_t i=0;i<rootObj["BaseOffset"].as_array().size();i++){
        auto element = JsonParser::JsonArray2EigenMatrixXd(rootObj["BaseOffset"].as_array()[i].as_array());
        baseOffset.push_back(element);
    }
    config.baseOffset = baseOffset;

    // For TargetOffset
    std::vector<Eigen::Matrix4d> targetOffset;
    for(size_t i=0;i<rootObj["targetOffset"].as_array().size();i++){
        auto element = JsonParser::JsonArray2EigenMatrixXd(rootObj["targetOffset"].as_array()[i].as_array());
        targetOffset.push_back(element);
    }
    config.targetOffset = targetOffset;

    return ArmSolver::GetPtr(config);
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
