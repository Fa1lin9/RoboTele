#include <iostream>
#include <RobotTeleoperate/RobotTeleoperate.hpp>
#include <thread>

int main(){
//    // IKSolver
//    Eigen::Matrix4d baseOffset;
//    baseOffset << 1, 0, 0, +0.02,
//                    0, 1, 0, 0,
//                    0, 0, 1, +1.10,
//                    0, 0 ,0, 1;
//    ArmSolver::BasicConfig solverConfig = {
//        .type = ArmSolver::Type::Ti5DualArm,
//        .baseFrameName = {"BASE_S"},
//        .targetFrameName = {"L_WRIST_R", "R_WRIST_R"},
//        .baseOffset = {baseOffset},
//        // for nlopt
////        .maxIteration = 400,
////        .relativeTol = 1e-2,
//        // for ipopt
//        .maxIteration = 50,
//        .relativeTol = 1e-6,
//        .armActiveDof = std::vector<int>{6, 6},
//    };

//    // Transform
//    Eigen::Matrix4d temp;
//    temp<<1,0,0,0,
//         0,-1,0,0,
//         0,0,-1,0,
//         0,0,0,1;
//    Transform::BasicConfig transformConfig;
//    transformConfig.type = Transform::Type::VisionPro2Ti5Robot;
//    transformConfig.T_Head2Waist = Eigen::Matrix4d::Identity();
//    transformConfig.T_XR2Robot <<   0, 0, -1, 0,
//                                    -1, 0, 0, 0,
//                                    0, 1, 0, 0,
//                                    0, 0, 0, 1;
//    transformConfig.T_Robot2LeftWrist <<0.0, 1.0, 0.0, 0.0,
//                                        -1.0, 0.0,0.0, 0.0,
//                                        0.0, 0.0, 1.0, 0.0,
//                                        0.0, 0.0, 0.0, 1.0;
////    transformConfig.T_Robot2LeftWrist = Eigen::Matrix4d::Identity();
////    transformConfig.T_Robot2LeftWrist = temp * transformConfig.T_Robot2LeftWrist;
//    transformConfig.T_Robot2RightWrist <<   0.0,-1.0, 0.0, 0.0,
//                                            1.0, 0.0, 0.0, 0.0,
//                                            0.0, 0.0, 1.0, 0.0,
//                                            0.0, 0.0, 0.0, 1.0;
////    transformConfig.T_Robot2RightWrist = Eigen::Matrix4d::Identity();
//    transformConfig.offset << 0, 0, 0;

//    // RobotHardware
//    RobotHardware::BasicConfig robotConfig = {
//        .robotType = RobotBase::RobotType::Ti5Robot,
//    };

//    // Ros2Bridge
//    Ros2Bridge::BasicConfig bridgeConfig;
//    bridgeConfig.msgType = Ros2Bridge::MsgType::JointStateWithoutStamp;
//    bridgeConfig.topicName = "position_control/joint_state";

//    /*************************************************************/

//    RobotTeleoperate::BasicConfig config = {
//        .robotType = RobotBase::RobotType::Ti5Robot,
//        .address = "tcp://127.0.0.1:5555",
////        .address = "ipc:///tmp/teleoperate",
//        .FPS = 25,
////        .solverConfig = solverConfig,
//        .robotConfig = robotConfig,
//        .transformConfig = transformConfig,
//        .bridgeConfig = bridgeConfig,
//        .isSim = true,
//        .isReal = false,
//    };

    std::string Ti5RobotConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/Ti5Robot.json";
    std::string CrpRobotConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/CrpRobot.json";
    std::string UnitreeG1Dof29ConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/UnitreeG1Dof29.json";
    std::string UnitreeG1Dof23ConfigPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/UnitreeG1Dof23.json";

    std::cout<<"Ti5Robot ConfigPath: "<<Ti5RobotConfigPath<<std::endl;
    std::cout<<"UnitreeG1Dof29 ConfigPath: "<<UnitreeG1Dof29ConfigPath<<std::endl;
    std::cout<<"UnitreeG1Dof23 ConfigPath: "<<UnitreeG1Dof23ConfigPath<<std::endl;

//    auto teleoperatePtr = RobotTeleoperate::GetPtr(Ti5RobotConfigPath);
    auto teleoperatePtr = RobotTeleoperate::GetPtr(CrpRobotConfigPath);
//    auto teleoperatePtr = RobotTeleoperate::GetPtr(UnitreeG1Dof23ConfigPath);

    teleoperatePtr->StartTeleop(false);
}
