#include <iostream>

#include <RobotHardware/RobotHardware.hpp>
#include <boost/shared_ptr.hpp>
#include <math.h>

int main(){

//    // Set configeration
//    RobotHardware::BasicConfig config = {
//        .robotType = RobotBase::RobotType::Ti5Robot,
//    };

//    boost::shared_ptr<RobotHardware> physicalRobotPtr
//            = RobotHardware::GetPtr(config);

//    physicalRobotPtr->Connect();

////    sleep(1);
//    std::cout<<"start to back to zero"<<std::endl;

////    physicalRobotPtr->BackToZero();

////    physicalRobotPtr->Info();

////    physicalRobotPtr->isConnect();
////    sleep(3);
////    std::cout<<" start to move !!! "<<std::endl;
//    // closely
//    // joint1: 0.03 -- 45
//    // joint2: 0.03 -- 45
//    // joint3: 0.06 -- 90
//    // joint4: 0.03 -- 45
//    // joint5:
//    // joint6: 0.02 -- 45
//    // joint7: 0.03 -- 45

////    std::vector<double> jointsAngle_ = { 0.01 , 0.02 , 0.02 , 0.01 , 0 , 0.02, 0.02};
////    physicalRobotPtr->MoveJ(jointsAngle_);

//    PhysicalRobot::Ti5RobotConfig crpRobotConfig = {
//        .useLeftArm = true,
////        .useRightArm = true,
////        .leftArmJointsValue = std::vector<double>{ 0 , -0.3 , 0.4 , 0.3 , 0 , 0.3 , 0.4 },
//        .leftArmJointsValue = std::vector<double>{ 0 , 0 , 0 , 0 , 0 , 0 , 0 },
//        .rightArmJointsValue = std::vector<double>{ 0 , 0 , 0 , 0 , 0 , 0 , 0 },
//    };

////    physicalRobotPtr->MoveJ(crpRobotConfig);

////    sleep(3);

//    std::cout<<"q: "<<physicalRobotPtr->GetJointsAngleEigen()<<std::endl;

////    physicalRobotPtr->GetJointsStatus();

//    physicalRobotPtr->Disconnect();
    RobotHardware::BasicConfig config  = {
        .networkInterface = "enp5s0",
        .robotType = RobotBase::RobotType::UnitreeG1Dof23,
        .description = "This is UnitreeG1Dof23",
        .headDof = 0,
        .armDof = 5,
        .waistDof = 1,
        .legDof = 6,
        .totalDof = 23,
    };

    auto hardware = RobotHardware::GetPtr(config);

    std::array<float, 13> init_pos{0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0,
                                   0, 0, 0};

    std::array<float, 13> target_pos = {0.f, M_PI_2,  0.f, M_PI_2, 0.f,
                                       0.f, -M_PI_2, 0.f, M_PI_2, 0.f,
                                       0.f, 0.f, 0.f};

    float control_dt = 0.02f;
    float max_joint_velocity = 0.5f;

    while(true){
        RobotHardware::HumanoidCmd cmd = {
            .isHeadEnabled = false,
            .isLeftArmEnabled = true,
            .isRightArmEnabled = true,
            .isWaistEnabled = false,
            .isLeftLegEnabled = false,
            .isRightLegEnabled = false,
            .qTargetLeftArm = std::vector<double>{0, 0, 0, 0, 0},
            .qTargetRightArm = std::vector<double>{0, 0, 0, 0, 0},
        };

        hardware->SendCmd(cmd);
    }
}
