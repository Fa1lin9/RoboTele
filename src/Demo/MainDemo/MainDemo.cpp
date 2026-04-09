#include <iostream>
#include <RobotTeleoperate/RobotTeleoperate.hpp>
#include <thread>

int main(){
    std::string Ti5RobotConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/Ti5Robot.json";
    std::string CrpRobotConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/CrpRobot.json";
    std::string UnitreeG1Dof29ConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/UnitreeG1Dof29.json";
    std::string UnitreeG1Dof23ConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/UnitreeG1Dof23.json";
    std::string UnitreeH1ConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/UnitreeH1.json";
    std::string FourierGR1ConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/FourierGR1.json";
    std::string AgiBotX1ConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/AgiBotX1.json";
    std::string FourierN1ConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/FourierN1.json";

    std::cout<<"Ti5Robot ConfigPath: "<<Ti5RobotConfigPath<<std::endl;
    std::cout<<"UnitreeG1Dof29 ConfigPath: "<<UnitreeG1Dof29ConfigPath<<std::endl;
    std::cout<<"UnitreeG1Dof23 ConfigPath: "<<UnitreeG1Dof23ConfigPath<<std::endl;

//    auto teleoperatePtr = RobotTeleoperate::GetPtr(Ti5RobotConfigPath);
//    auto teleoperatePtr = RobotTeleoperate::GetPtr(CrpRobotConfigPath);
//    auto teleoperatePtr = RobotTeleoperate::GetPtr(UnitreeG1Dof23ConfigPath);
//    auto teleoperatePtr = RobotTeleoperate::GetPtr(UnitreeH1ConfigPath);
//    auto teleoperatePtr = RobotTeleoperate::GetPtr(FourierGR1ConfigPath);
//    auto teleoperatePtr = RobotTeleoperate::GetPtr(AgiBotX1ConfigPath);
    auto teleoperatePtr = RobotTeleoperate::GetPtr(FourierN1ConfigPath);


    teleoperatePtr->StartTeleop(false);
}
