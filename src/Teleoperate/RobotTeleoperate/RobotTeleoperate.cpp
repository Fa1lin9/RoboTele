#include <RobotTeleoperate/RobotTeleoperate.hpp>
#include <Ti5RobotTeleoperate/Ti5RobotTeleoperate.hpp>
#include <UnitreeG1Teleoperate/UnitreeG1Teleoperate.hpp>
#include <GenericTeleoperate/GenericTeleoperate.hpp>

RobotTeleoperate::RobotTeleoperate(){

}

RobotTeleoperate::~RobotTeleoperate(){

}

std::shared_ptr<RobotTeleoperate> RobotTeleoperate::GetPtr(const RobotTeleoperate::BasicConfig &config_){
    switch (config_.robotType) {
        case RobotBase::RobotType::Ti5Robot :{
//           return std::make_shared<Ti5RobotTeleoperate>(config_);
            return std::make_shared<GenericTeleoperate>(config_);
        }
        case RobotBase::RobotType::UnitreeG1Dof29 :{
//           return std::make_shared<UnitreeG1Teleoperate>(config_);
           return std::make_shared<GenericTeleoperate>(config_);
        }
        case RobotBase::RobotType::UnitreeG1Dof23 :{
//           return std::make_shared<UnitreeG1Teleoperate>(config_);
           return std::make_shared<GenericTeleoperate>(config_);
        }
        case RobotBase::RobotType::CrpRobot :{
//           return std::make_shared<Ti5RobotTeleoperate>(config_);
           return std::make_shared<GenericTeleoperate>(config_);
        }
        case RobotBase::RobotType::GenericRobot :{
           return std::make_shared<GenericTeleoperate>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

std::shared_ptr<RobotTeleoperate> RobotTeleoperate::GetPtr(const std::string& filePath){
    JsonParser jsonParser(filePath);
    json::object rootObj = jsonParser.GetJsonObject();

//    json::object solverObj = rootObj["SolverConfig"].as_object();
//    json::object transformObj = rootObj["TransformConfig"].as_object();
//    json::object hardwareObj = rootObj["HardwareConfig"].as_object();
    json::object ros2BridgeObj = rootObj["BridgeConfig"].as_object();
    json::object configPathObj = rootObj["ConfigPath"].as_object();
    json::object bodyEnableObj = rootObj["BodyEnable"].as_object();

    RobotBase::RobotType robotType = RobotBase::GetTypeFromStr(rootObj["RobotType"].as_string().c_str());

    // Ros2Bridge
    // Body
    Ros2Bridge::BasicConfig bodyBridgeConfig = {
        .topicName = ros2BridgeObj["Body"].as_object()["TopicName"].as_string().c_str(),
        .msgType =
        Ros2Bridge::GetMsgTypeFromStr(ros2BridgeObj["Body"].as_object()["MsgType"].as_string().c_str()),
    };
    // Left Hand
    Ros2Bridge::BasicConfig leftHandBridgeConfig = {
        .topicName = ros2BridgeObj["LeftHand"].as_object()["TopicName"].as_string().c_str(),
        .msgType =
        Ros2Bridge::GetMsgTypeFromStr(ros2BridgeObj["LeftHand"].as_object()["MsgType"].as_string().c_str()),
    };
    // Right Hand
    Ros2Bridge::BasicConfig rightHandBridgeConfig = {
        .topicName = ros2BridgeObj["RightHand"].as_object()["TopicName"].as_string().c_str(),
        .msgType =
        Ros2Bridge::GetMsgTypeFromStr(ros2BridgeObj["RightHand"].as_object()["MsgType"].as_string().c_str()),
    };

    RobotTeleoperate::BasicConfig config = {
        .robotType = robotType,
        .xrType = XRBase::GetTypeFromStr(rootObj["XRType"].as_string().c_str()),
        .handType = HandBase::GetTypeFromStr(rootObj["HandType"].as_string().c_str()),

        .address = rootObj["Address"].as_string().c_str(),
        .FPS = static_cast<int>(rootObj["FPS"].as_int64()),

//        .armSolverConfigPath = configPathObj["ArmSolverConfigPath"].as_string().c_str(),
//        .headSolverConfigPath = configPathObj["HeadSolverConfigPath"].as_string().c_str(),
//        .waistSolverConfigPath = configPathObj["WaistSolverConfigPath"].as_string().c_str(),
//        .hardwareConfigPath = configPathObj["HardwareConfigPath"].as_string().c_str(),
//        .transformConfigPath = configPathObj["TransformConfigPath"].as_string().c_str(),

        .bodyBridgeConfig = bodyBridgeConfig,
        .leftHandBridgeConfig = leftHandBridgeConfig,
        .rightHandBridgeConfig = rightHandBridgeConfig,

        .isSim  = rootObj["IsSimulatedRobot"].as_bool(),
        .isReal = rootObj["IsRealWorldRobot"].as_bool(),

        .isCheckSolution = rootObj["IsCheckSolution"].as_bool(),
        .isFilterSolution = rootObj["IsFilterSolution"].as_bool(),

//        .enableHead = rootObj["EnableHead"].as_bool(),
//        .enableLeftArm = rootObj["EnableLeftArm"].as_bool(),
//        .enableRightArm = rootObj["EnableRightArm"].as_bool(),
//        .enableWaist = rootObj["EnableWaist"].as_bool(),
//        .enableLeftLeg = rootObj["EnableLeftLeg"].as_bool(),
//        .enableRightLeg = rootObj["EnableRightLeg"].as_bool(),

        .useRootPath = configPathObj["UseRootPath"].as_bool(),
    };

    // For Filter
    Eigen::VectorXd filterWeightEigen = JsonParser::JsonArray2EigenVectorXd(rootObj["FilterWeight"].as_array());
    config.filterWeight = std::vector<double>(filterWeightEigen.data(),
                                              filterWeightEigen.data() + filterWeightEigen.size());

    // For ConfigPath
    std::string rootPath = static_cast<std::string>(SOURCE_FILE_PATH);

    auto getFullPath = [&](const char* key){
        std::string p = configPathObj[key].as_string().c_str();
        return config.useRootPath ? (rootPath + p) : p;
    };

    config.armSolverConfigPath     = getFullPath("ArmSolver");
    config.headSolverConfigPath    = getFullPath("HeadSolver");
    config.waistSolverConfigPath   = getFullPath("WaistSolver");
    config.hardwareConfigPath      = getFullPath("RobotHardware");
    config.transformConfigPath     = getFullPath("Transform");

    // For BodyEnable
    config.enableHead     = bodyEnableObj["Head"].as_bool();
    config.enableWaist    = bodyEnableObj["Waist"].as_bool();
    config.enableLeftArm  = bodyEnableObj["LeftArm"].as_bool();
    config.enableRightArm = bodyEnableObj["RightArm"].as_bool();
    config.enableLeftLeg  = bodyEnableObj["LeftLeg"].as_bool();
    config.enableRightLeg = bodyEnableObj["RightLeg"].as_bool();
    config.enableLeftHand = bodyEnableObj["LeftHand"].as_bool();
    config.enableRightHand = bodyEnableObj["RightHand"].as_bool();

    return RobotTeleoperate::GetPtr(config);
}

bool RobotTeleoperate::Init()
{
    std::cout << "[RobotTeleoperate] Calling Init()." << std::endl;
    return true;
}

bool RobotTeleoperate::StartTeleop(bool verbose)
{
    std::cout << "[RobotTeleoperate] Calling StartTeleop()." << std::endl;
    return true;
}

bool RobotTeleoperate::StopTeleop()
{
    std::cout << "[RobotTeleoperate] Calling StopTeleop()." << std::endl;
    return true;
}
