#include <ArmSolver/ArmSolver.hpp>
#include <Ti5DualArmSolver/Ti5DualArmSolver.hpp>
#include <G1Dof23DualArmSolver/G1Dof23DualArmSolver.hpp>
#include <G1Dof29DualArmSolver/G1Dof29DualArmSolver.hpp>
#include <GenericDualArmSolver/GenericDualArmSolver.hpp>

ArmSolver::ArmSolver(){

}

ArmSolver::~ArmSolver(){

}

std::vector<std::string> ArmSolver::GetLeftArmJointNames()
{
    std::cout << "[ArmSolver::GetLeftArmJointNames] Empty virtual function called. No implementation provided." << std::endl;
    return {};
}

std::vector<std::string> ArmSolver::GetRightArmJointNames()
{
    std::cout << "[ArmSolver::GetRightArmJointNames] Empty virtual function called. No implementation provided." << std::endl;
    return {};
}

std::vector<size_t> ArmSolver::GetLeftArmJointIndex()
{
    std::cout << "[ArmSolver::GetLeftArmJointIndex] Empty virtual function called. No implementation provided." << std::endl;
    return {};
}

std::vector<size_t> ArmSolver::GetRightArmJointIndex()
{
    std::cout << "[ArmSolver::GetRightArmJointIndex] Empty virtual function called. No implementation provided." << std::endl;
    return {};
}

std::vector<size_t> ArmSolver::GetLeftArmQIndex()
{
    std::cout << "[ArmSolver::GetLeftArmQIndex] Empty virtual function called. No implementation provided." << std::endl;
    return {};
}

std::vector<size_t> ArmSolver::GetRightArmQIndex()
{
    std::cout << "[ArmSolver::GetRightArmQIndex] Empty virtual function called. No implementation provided." << std::endl;
    return {};
}


std::shared_ptr<ArmSolver> ArmSolver::GetPtr(const ArmSolver::BasicConfig &config_){
    switch (config_.type) {
        case ArmSolver::Type::Ti5DualArm :{
           return std::make_shared<Ti5DualArmSolver>(config_);
        }
        case ArmSolver::Type::G1Dof23DualArm :{
           return std::make_shared<G1Dof23DualArmSolver>(config_);
        }
        case ArmSolver::Type::G1Dof29DualArm :{
           return std::make_shared<G1Dof29DualArmSolver>(config_);
        }
        case ArmSolver::Type::GenericDualArm :{
           return std::make_shared<GenericDualArmSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

std::shared_ptr<ArmSolver> ArmSolver::GetPtr(const std::string& filePath){
    JsonParser jsonParser(filePath);
    json::object rootObj = jsonParser.GetJsonObject();

    // ArmSolver
    ArmSolver::BasicConfig config = {
        .useRootPath = rootObj["UseRootPath"].as_bool(),
//        .modelPath = rootObj["ModelPath"].as_string().c_str(),

        .type = ArmSolver::GetTypeFromStr(rootObj["Type"].as_string().c_str()),
        .robotType = RobotBase::GetTypeFromStr(rootObj["RobotType"].as_string().c_str()),

        .baseFrameName = JsonParser::JsonArray2StdVecStr(rootObj["BaseFrameName"].as_array()),
        .targetFrameName = JsonParser::JsonArray2StdVecStr(rootObj["TargetFrameName"].as_array()),

        .leftArmJointNames = JsonParser::JsonArray2StdVecStr(rootObj["LeftArmJointNames"].as_array()),
        .rightArmJointNames = JsonParser::JsonArray2StdVecStr(rootObj["RightArmJointNames"].as_array()),

        .maxIteration = static_cast<int>(rootObj["MaxIteration"].as_int64()),
        .relativeTol = rootObj["RelativeTol"].as_double(),
        .armActiveDof = JsonParser::JsonArray2StdVecInt(rootObj["ArmActiveDof"].as_array()),
        .wTranslation = rootObj["ObjectiveFunc"].as_object()["TranslationWeight"].as_double(),
        .wRotation = rootObj["ObjectiveFunc"].as_object()["RotationWeight"].as_double(),
        .wRegularization = rootObj["ObjectiveFunc"].as_object()["RegularizationWeight"].as_double(),
        .wSmooth = rootObj["ObjectiveFunc"].as_object()["SmoothWeight"].as_double(),
    };
    // For Model Path
    std::string rootPath = static_cast<std::string>(SOURCE_FILE_PATH);
    std::string modelPath = rootObj["ModelPath"].as_string().c_str();

    if(config.useRootPath){
        config.modelPath = rootPath + modelPath;
    }else{
        config.modelPath = rootPath;
    }

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
    for(size_t i=0;i<rootObj["TargetOffset"].as_array().size();i++){
        auto element = JsonParser::JsonArray2EigenMatrixXd(rootObj["TargetOffset"].as_array()[i].as_array());
        targetOffset.push_back(element);
    }
    config.targetOffset = targetOffset;

    return ArmSolver::GetPtr(config);
}

const std::unordered_map<std::string, ArmSolver::Type> ArmSolver::typeMap = {
    {"Ti5DualArm", ArmSolver::Type::Ti5DualArm},
    {"G1Dof23DualArm", ArmSolver::Type::G1Dof23DualArm},
    {"G1Dof29DualArm", ArmSolver::Type::G1Dof29DualArm},
    {"GenericDualArm", ArmSolver::Type::GenericDualArm},
};

ArmSolver::Type ArmSolver::GetTypeFromStr(const std::string& str){
    auto temp = ArmSolver::typeMap.find(str);
    if(temp != ArmSolver::typeMap.end()){
        return temp->second;
    }

    throw std::invalid_argument("[RobotType::GetTypeFromStr] Invalid string");
}
