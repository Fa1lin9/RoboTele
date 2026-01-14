#include <iostream>

#include <RobotHardware/RobotHardware.hpp>
#include <boost/shared_ptr.hpp>
#include <math.h>
#include <thread>

#include <source_path.h>
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

//    auto hardware = RobotHardware::GetPtr(config);

    std::string configPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/RobotHardware/UnitreeG1Dof23.json";
    auto hardware = RobotHardware::GetPtr(configPath);

    while(true){
        auto start = std::chrono::high_resolution_clock::now();
        std::cout << "-----------------------------------" << std::endl;
        RobotHardware::HumanoidCmd cmd = {
            .enableHead = false,
            .enableLeftArm = true,
            .enableRightArm = true,
            .enableWaist = false,
            .enableLeftLeg = false,
            .enableRightLeg = false,
            .qTargetLeftArm = std::vector<double>{0., 0, 0, 0.5, 0},
            .qTargetRightArm = std::vector<double>{0., 0, 0, 0.5, 0},
        };
        auto state = hardware->GetState(true);
        hardware->SendCmd(cmd);

        auto temp = hardware->GetJointsAngleEigen();

        // Sleep
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << " Main Loop 耗时: " << duration.count() << " ms" << std::endl;

        double FPS = 25.0;
        int framePeriod = static_cast<int>(1000.0 / FPS);
        int sleepTime = framePeriod - duration.count();

        if(sleepTime > 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
        }
    }
}
