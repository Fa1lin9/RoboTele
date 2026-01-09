#include <RobotHardware/RobotHardware.hpp>
#include <Ti5RobotHardware/Ti5RobotHardware.hpp>
#include <G1Dof23Hardware/G1Dof23Hardware.hpp>
//#include <G1Dof29Hardware/G1Dof29Hardware.hpp>

RobotHardware::RobotHardware(){

}

RobotHardware::~RobotHardware(){

}

/* ---------------- Connection ---------------- */

bool RobotHardware::Connect() {
    std::cout << "[RobotHardware] Calling Connect()." << std::endl;
    return true;
}

bool RobotHardware::Disconnect() {
    std::cout << "[RobotHardware] Calling Disconnect()." << std::endl;
    return true;
}

bool RobotHardware::isConnect() {
    std::cout << "[RobotHardware] Calling isConnect()." << std::endl;
    return true;
}


/* ---------------- Basic Action ---------------- */

bool RobotHardware::Init() {
    std::cout << "[RobotHardware] Calling Init()." << std::endl;
    return true;
}

bool RobotHardware::EmergencyStop() {
    std::cout << "[RobotHardware] Calling EmergencyStop()." << std::endl;
    return true;
}

bool RobotHardware::BackToZero() {
    std::cout << "[RobotHardware] Calling BackToZero()." << std::endl;
    return true;
}

bool RobotHardware::MoveJ(const std::vector<double> &jointsAngle_) {
    std::cout << "[RobotHardware] Calling MoveJ()." << std::endl;
    return true;
}

bool RobotHardware::MoveL() {
    std::cout << "[RobotHardware] Calling MoveL()." << std::endl;
    return true;
}

/* ---------------- Get Information ---------------- */

std::vector<double> RobotHardware::GetJointsAngle() {
    std::cout << "[RobotHardware] Calling GetJointsAngle()." << std::endl;
    return std::vector<double>();
}

Eigen::VectorXd RobotHardware::GetJointsAngleEigen() {
    std::cout << "[RobotHardware] Calling GetJointsAngleEigen()." << std::endl;
    return Eigen::VectorXd();
}

void RobotHardware::Info() {
    std::cout << "[RobotHardware] Calling Info()." << std::endl;
}

void RobotHardware::GetJointsStatus() {
    std::cout << "[RobotHardware] Calling GetJointsStatus()." << std::endl;
}

bool RobotHardware::MoveJ(const HumanoidCmd &cmd)
{
    std::cout << "[RobotHardware] Calling MoveJ()." << std::endl;
    return true;
}

RobotHardware::HumanoidState RobotHardware::GetState()
{
    std::cout << "[RobotHardware] Calling GetState()." << std::endl;
    return HumanoidState();
}

bool RobotHardware::SendCmd(const HumanoidCmd &cmd)
{
    std::cout << "[RobotHardware] Calling SendCmd()." << std::endl;
    return true;
}

boost::shared_ptr<RobotHardware> RobotHardware::GetPtr(const RobotHardware::BasicConfig &config){
    switch (config.robotType) {
        case RobotBase::RobotType::Ti5Robot :{
           return boost::make_shared<Ti5RobotHardware>(config);
        }
        case RobotBase::RobotType::UnitreeG1Dof23 :{
           return boost::make_shared<G1Dof23Hardware>(config);
        }
//        case RobotBase::RobotType::UnitreeG1Dof29 :{
//           return boost::make_shared<G1Dof29Hardware>(config);
//        }
        default:{
            return nullptr;
        }
    }
}

boost::shared_ptr<RobotHardware> RobotHardware::GetPtr(const std::string& filePath){
    JsonParser jsonParser(filePath);
    json::object rootObj = jsonParser.GetJsonObject();

    // RobotHardware
    RobotHardware::BasicConfig config = {
        .IP = rootObj["IP"].as_string().c_str(),
        .networkInterface = rootObj["NetworkInterface"].as_string().c_str(),
        .robotType = RobotBase::GetTypeFromStr(rootObj["RobotType"].as_string().c_str()),
        .description = rootObj["Description"].as_string().c_str(),
        .headDof = static_cast<size_t>(rootObj["HeadDof"].as_int64()),
        .armDof = static_cast<size_t>(rootObj["ArmDof"].as_int64()),
        .waistDof = static_cast<size_t>(rootObj["WaistDof"].as_int64()),
        .legDof = static_cast<size_t>(rootObj["LegDof"].as_int64()),
        .totalDof = static_cast<size_t>(rootObj["TotalDof"].as_int64()),
    };

    return RobotHardware::GetPtr(config);
}
