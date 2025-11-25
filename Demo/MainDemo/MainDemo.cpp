#include <iostream>
#include <RobotTeleoperate/RobotTeleoperate.hpp>
#include <Ti5RobotTeleoperate/Ti5RobotTeleoperate.hpp>
#include <thread>

int main(){
    // IKSolver
    Eigen::Matrix4d baseOffset;
    baseOffset << 1, 0, 0, +0.02,
                    0, 1, 0, 0,
                    0, 0, 1, +1.10,
                    0, 0 ,0, 1;
    ArmSolver::BasicConfig solverConfig = {
        .type = ArmSolver::Type::Ti5DualArm,
        .baseFrameName = {"BASE_S"},
        .targetFrameName = {"L_WRIST_R", "R_WRIST_R"},
        .baseOffset = {baseOffset},
        // for nlopt
//        .maxIteration = 400,
//        .relativeTol = 1e-2,
        // for ipopt
        .maxIteration = 50,
        .relativeTol = 1e-6,
    };
    Eigen::VectorXd v(2);
    v << 6, 6;
    solverConfig.dofArm = v;

    // CoordinateTransform
    Eigen::Matrix4d temp;
    temp<<1,0,0,0,
         0,-1,0,0,
         0,0,-1,0,
         0,0,0,1;
    CoordinateTransform::BasicConfig transformConfig;
    transformConfig.type = CoordinateTransform::Type::VisionPro2Ti5Robot;
    transformConfig.T_Head2Waist = Eigen::Matrix4d::Identity();
    transformConfig.T_XR2Robot <<   0, 0, -1, 0,
                                    -1, 0, 0, 0,
                                    0, 1, 0, 0,
                                    0, 0, 0, 1;
    transformConfig.T_Robot2LeftWrist <<0.0, 1.0, 0.0, 0.0,
                                        -1.0, 0.0,0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0;
//    transformConfig.T_Robot2LeftWrist = Eigen::Matrix4d::Identity();
//    transformConfig.T_Robot2LeftWrist = temp * transformConfig.T_Robot2LeftWrist;
    transformConfig.T_Robot2RightWrist <<   0.0,-1.0, 0.0, 0.0,
                                            1.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0, 0.0,
                                            0.0, 0.0, 0.0, 1.0;
//    transformConfig.T_Robot2RightWrist = Eigen::Matrix4d::Identity();
    transformConfig.offset << 0, 0, 0;

    // PhysicalRobot
    PhysicalRobot::BasicConfig robotConfig = {
        .robotType = RobotType::Type::Ti5Robot,
    };

    // Ros2Bridge
    Ros2Bridge::BasicConfig bridgeConfig;
    bridgeConfig.msgType = Ros2Bridge::MsgType::JointStateWithoutStamp;
    bridgeConfig.topicName = "position_control/joint_state";

    /*************************************************************/

    RobotTeleoperate::BasicConfig config = {
        .robotType = RobotType::Type::Ti5Robot,
        .address = "tcp://127.0.0.1:5555",
//        .address = "ipc:///tmp/teleoperate",
        .FPS = 25,
        .solverConfig = solverConfig,
        .robotConfig = robotConfig,
        .transformConfig = transformConfig,
        .bridgeConfig = bridgeConfig,
        .isSim = true,
        .isReal = false,
    };

    std::string configPath = static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/Ti5Robot.json";

    std::cout<<"configPath: "<<configPath<<std::endl;

//    auto teleoperatePtr = RobotTeleoperate::GetPtr(config);
    auto teleoperatePtr = RobotTeleoperate::GetPtr(configPath);

    teleoperatePtr->StartTeleoperate(false);
}
