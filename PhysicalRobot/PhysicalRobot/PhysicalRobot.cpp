#include <PhysicalRobot/PhysicalRobot.hpp>
#include <Ti5PhysicalRobot/Ti5PhysicalRobot.hpp>

PhysicalRobot::PhysicalRobot(){

}

PhysicalRobot::~PhysicalRobot(){

}

bool PhysicalRobot::BackToInitPose(const PhysicalRobot::Ti5RobotConfig& config_){
    LOG_FUNCTION;
    return true;
}

bool PhysicalRobot::BackToZero(const PhysicalRobot::Ti5RobotConfig& config_){
    LOG_FUNCTION;
    return true;
}

bool PhysicalRobot::MoveJ(const PhysicalRobot::Ti5RobotConfig& config_){
    LOG_FUNCTION;
    return true;
}

boost::shared_ptr<PhysicalRobot> PhysicalRobot::GetPtr(const PhysicalRobot::BasicConfig &config_){
    switch (config_.robotType) {
        case RobotBase::RobotType::Ti5Robot :{
           return boost::make_shared<Ti5PhysicalRobot>(config_);
        }
        default:{
            return nullptr;
        }
    }
}
