#include <DataCollector/DataCollector.hpp>
#include <HandSolver/HandSolver.hpp>
#include <HandController/HandController.hpp>
#include <Ros2Bridge/Ros2Bridge.h>

double ThumbFingerMap(const double& x){
    return 0.010 * (36 - x) / 34;
}

double OtherFingerMap(const double& x){
    return 0.019 * (178 - x) / 78;
}

int FPS = 25;

int main(){
    DataCollector dataCollector("tcp://127.0.0.1:5555");

    HandSolver::BasicConfig handSolverConfig;
    handSolverConfig.dofHand = 12;
    handSolverConfig.type = XRBase::XRType::VisionPro;
    auto handSolver = HandSolver::GetPtr(handSolverConfig);

    // Ros2
    Ros2Bridge bridge;
    Ros2Bridge::BasicConfig bridgeConfig;
    bridgeConfig.msgType = Ros2Bridge::MsgType::JointStateWithoutStamp;
    bridgeConfig.topicName = "rohand_left/external_external_joint_states";
    bridge.Init(bridgeConfig);

    // Collector VisionPro's Data
    std::thread dataThread(&DataCollector::Run, &dataCollector);

    HandBase::DualHandData dualHandData;
    while(true){        
        if(dataCollector.HasNewData()){
            std::cout << "Get New Data! " << std::endl;
            dualHandData.leftHandData.handPositions = dataCollector.GetLeftHandPositions();
            dualHandData.rightHandData.handPositions = dataCollector.GetRightHandPositions();
            dualHandData.type = HandBase::HandType::ROHand;

            if(dualHandData.leftHandData.handPositions[0].sum() == 0 ||
               dualHandData.rightHandData.handPositions[0].sum() == 0){
                continue;
            }
        }else{
            continue;
        }

        auto start = std::chrono::high_resolution_clock::now();
//        std::cout <<"Start Time: "
//                  <<std::chrono::time_point_cast<std::chrono::milliseconds>(start).time_since_epoch().count()
//                  <<std::endl;
        auto handAngle = handSolver->SolveDualHand(dualHandData);

        std::cout << "-----------------------------------------" << std::endl;
        std::cout << "HandAngle: " << std::endl;
        std::cout << handAngle << std::endl;

        ti5_interfaces::msg::JointStateWithoutStamp msg;
        std::vector<double> position;
        position.push_back(OtherFingerMap(handAngle(1)));
        position.push_back(OtherFingerMap(handAngle(2)));
        position.push_back(OtherFingerMap(handAngle(3)));
        position.push_back(OtherFingerMap(handAngle(4)));
        position.push_back(ThumbFingerMap(handAngle(0)));
        position.push_back(handAngle(5)*M_PI/180.0);
        msg.position() = position;
        msg.name() = std::vector<std::string>{  "if_slider_link",
                                                "mf_slider_link",
                                                "rf_slider_link",
                                                "lf_slider_link",
                                                "th_slider_link",
                                                "th_root_link"};
        bridge.SendMsg(msg);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << " Main Loop 耗时: " << duration.count() << " ms" << std::endl;

        int framePeriod = static_cast<int>(1000.0 / FPS);
        int sleepTime = framePeriod - duration.count();

        if(sleepTime > 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
        }
    }

    // delete the thread
    dataCollector.Stop();
    dataThread.join();
}
