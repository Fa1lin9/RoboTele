#include <RobotTeleoperate/RobotTeleoperate.hpp>
#include <Ti5RobotTeleoperate/Ti5RobotTeleoperate.hpp>
#include <UnitreeG1Teleoperate/UnitreeG1Teleoperate.hpp>

RobotTeleoperate::RobotTeleoperate(){

}

RobotTeleoperate::~RobotTeleoperate(){

}

boost::shared_ptr<RobotTeleoperate> RobotTeleoperate::GetPtr(const RobotTeleoperate::BasicConfig &config_){
    switch (config_.robotType) {
        case RobotBase::RobotType::Ti5Robot :{
           return boost::make_shared<Ti5RobotTeleoperate>(config_);
        }
        case RobotBase::RobotType::UnitreeG1Dof29 :{
           return boost::make_shared<UnitreeG1Teleoperate>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

boost::shared_ptr<RobotTeleoperate> RobotTeleoperate::GetPtr(const std::string& filePath){
    JsonParser jsonParser(filePath);
    json::object rootObj = jsonParser.GetJsonObject();

    json::object solverObj = rootObj["SolverConfig"].as_object();
    json::object transformObj = rootObj["TransformConfig"].as_object();
    json::object physicalRObotObj = rootObj["PhysicalRobotConfig"].as_object();
    json::object ros2BridgeObj = rootObj["Ros2BridgeConfig"].as_object();

    RobotBase::RobotType robotType = RobotBase::GetTypeFromStr(rootObj["RobotType"].as_string().c_str());

//    std::cout << "Translation kind = " << (int)solverObj["TranslationWeight"].kind() << std::endl;
//    std::cout << "Rotation kind = " << (int)solverObj["RotationWeight"].kind() << std::endl;
//    std::cout << "Regularization kind = " << (int)solverObj["RegularizationWeight"].kind() << std::endl;
//    std::cout << "Smooth kind = " << (int)solverObj["SmoothWeight"].kind() << std::endl;

    // ArmSolver
    ArmSolver::BasicConfig solverConfig = {
        .type = ArmSolver::GetTypeFromStr(solverObj["Type"].as_string().c_str()),
//        .baseFrameName = {"BASE_S"},
        .baseFrameName = JsonParser::JsonArray2StdVecStr(solverObj["BaseFrameName"].as_array()),
//        .targetFrameName = {"L_WRIST_R", "R_WRIST_R"},
        .targetFrameName = JsonParser::JsonArray2StdVecStr(solverObj["TargetFrameName"].as_array()),
//        .baseOffset = {JsonParser::JsonArray2EigenMatrixXd(solverObj["BaseOffset"].as_array()[0].as_array())},
        // for nlopt
//        .maxIteration = 400,
//        .relativeTol = 1e-2,
        // for ipopt
        .maxIteration = static_cast<int>(solverObj["MaxIteration"].as_int64()),
        .relativeTol = solverObj["RelativeTol"].as_double(),
        .dofArm = JsonParser::JsonArray2StdVecInt(solverObj["DofArm"].as_array()),
        .wTranslation = solverObj["ObjectiveFunc"].as_object()["TranslationWeight"].as_double(),
        .wRotation = solverObj["ObjectiveFunc"].as_object()["RotationWeight"].as_double(),
        .wRegularization = solverObj["ObjectiveFunc"].as_object()["RegularizationWeight"].as_double(),
        .wSmooth = solverObj["ObjectiveFunc"].as_object()["SmoothWeight"].as_double(),
    };
    // In the future, the variable BaseOffset maybe not just 1
    // So I choose to set BaseOffset to 3-D array
    std::vector<Eigen::Matrix4d> baseOffset;
    for(size_t i=0;i<solverObj["BaseOffset"].as_array().size();i++){
        auto element = JsonParser::JsonArray2EigenMatrixXd(solverObj["BaseOffset"].as_array()[i].as_array());
        baseOffset.push_back(element);
    }
    solverConfig.baseOffset = baseOffset;


    // Transform
    Transform::BasicConfig transformConfig = {
        .T_Head2Waist = JsonParser::JsonArray2EigenMatrixXd(transformObj["T_Head2Waist"].as_array()),
        .T_XR2Robot = JsonParser::JsonArray2EigenMatrixXd(transformObj["T_XR2Robot"].as_array()),
        .T_Robot2LeftWrist = JsonParser::JsonArray2EigenMatrixXd(transformObj["T_Robot2LeftWrist"].as_array()),
        .T_Robot2RightWrist = JsonParser::JsonArray2EigenMatrixXd(transformObj["T_Robot2RightWrist"].as_array()),
        .offset = JsonParser::JsonArray2EigenVectorXd(transformObj["Offset"].as_array()),
        .type = Transform::GetTypeFromStr(transformObj["Type"].as_string().c_str()),
    };

    // PhysicalRobot
    PhysicalRobot::BasicConfig physicalRobotConfig = {
        .robotType = robotType,
    };

    // Ros2Bridge
    Ros2Bridge::BasicConfig bridgeConfig = {
        .topicName = ros2BridgeObj["TopicName"].as_string().c_str(),
        .msgType = Ros2Bridge::GetMsgTypeFromStr(ros2BridgeObj["MsgType"].as_string().c_str()),
    };


    RobotTeleoperate::BasicConfig config = {
        .robotType = robotType,
        .xrType = XRBase::GetTypeFromStr(rootObj["XRType"].as_string().c_str()),
        .address = rootObj["Address"].as_string().c_str(),
        .FPS = static_cast<int>(rootObj["FPS"].as_int64()),
        .solverConfig = solverConfig,
        .robotConfig = physicalRobotConfig,
        .transformConfig = transformConfig,
        .bridgeConfig = bridgeConfig,
        .isSim  = rootObj["IsSimulatedRobot"].as_bool(),
        .isReal = rootObj["IsRealWorldRobot"].as_bool(),
        .isCheckSolution = rootObj["IsCheckSolution"].as_bool(),
        .isFilterSolution = rootObj["IsFilterSolution"].as_bool(),
        .useHead = rootObj["UseHead"].as_bool(),
        .useWaist = rootObj["UseWaist"].as_bool(),
    };
    Eigen::VectorXd filterWeightEigen = JsonParser::JsonArray2EigenVectorXd(rootObj["FilterWeight"].as_array());
    config.filterWeight = std::vector<double>(filterWeightEigen.data(),
                                              filterWeightEigen.data() + filterWeightEigen.size());

    return RobotTeleoperate::GetPtr(config);
}
