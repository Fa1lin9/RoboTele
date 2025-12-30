#include <RobotHardware/RobotHardware.hpp>
#include <Ti5RobotHardware/Ti5RobotHardware.hpp>
#include <G1Dof23Hardware/G1Dof23Hardware.hpp>
#include <G1Dof29Hardware/G1Dof29Hardware.hpp>

RobotHardware::RobotHardware(){

}

RobotHardware::~RobotHardware(){

}

/* ---------------- Basic Action ---------------- */

bool RobotHardware::Init() {
    std::cout << "[PhysicalRobot] Calling Init()." << std::endl;
    return true;
}

bool RobotHardware::EmergencyStop() {
    std::cout << "[PhysicalRobot] Calling EmergencyStop()." << std::endl;
    return true;
}

bool RobotHardware::BackToZero() {
    std::cout << "[PhysicalRobot] Calling BackToZero()." << std::endl;
    return true;
}

bool RobotHardware::MoveJ(const std::vector<double> &jointsAngle_) {
    std::cout << "[PhysicalRobot] Calling MoveJ()." << std::endl;
    return true;
}

bool RobotHardware::MoveL() {
    std::cout << "[PhysicalRobot] Calling MoveL()." << std::endl;
    return true;
}

/* ---------------- Get Information ---------------- */

std::vector<double> RobotHardware::GetJointsAngle() {
    std::cout << "[PhysicalRobot] Calling GetJointsAngle()." << std::endl;
    return std::vector<double>();
}

Eigen::VectorXd RobotHardware::GetJointsAngleEigen() {
    std::cout << "[PhysicalRobot] Calling GetJointsAngleEigen()." << std::endl;
    return Eigen::VectorXd();
}

void RobotHardware::Info() {
    std::cout << "[PhysicalRobot] Calling Info()." << std::endl;
}

void RobotHardware::GetJointsStatus() {
    std::cout << "[PhysicalRobot] Calling GetJointsStatus()." << std::endl;
}

boost::shared_ptr<RobotHardware> RobotHardware::GetPtr(const RobotHardware::BasicConfig &config){
    switch (config.robotType) {
        case RobotBase::RobotType::Ti5Robot :{
           return boost::make_shared<Ti5RobotHardware>(config);
        }
        case RobotBase::RobotType::UnitreeG1Dof23 :{
           return boost::make_shared<G1Dof23Hardware>(config);
        }
        case RobotBase::RobotType::UnitreeG1Dof29 :{
           return boost::make_shared<G1Dof29Hardware>(config);
        }
        default:{
            return nullptr;
        }
    }
}
