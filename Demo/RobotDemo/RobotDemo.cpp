#include <iostream>

#include <RobotHardware/RobotHardware.hpp>
#include <boost/shared_ptr.hpp>
#include <math.h>
#include <thread>

#include <source_path.h>
int main(){
//    RobotHardware::BasicConfig config  = {
//        .networkInterface = "enp5s0",
//        .robotType = RobotBase::RobotType::UnitreeG1Dof23,
//        .description = "This is UnitreeG1Dof23",
//        .headDof = 0,
//        .armDof = 5,
//        .waistDof = 1,
//        .legDof = 6,
//        .totalDof = 23,
//    };

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
