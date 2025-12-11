#include <DataCollector/DataCollector.hpp>
#include <HandSolver/HandSolver.hpp>
#include <HandController/HandController.hpp>
#include <Ros2Bridge/Ros2Bridge.h>
#include <HandGestureDetector/HandGestureDetector.hpp>
#include <WeightedMovingFilter/WeightedMovingFilter.hpp>

double ThumbFingerMap(const double& x){
    return 0.010 * (36 - x) / 34;
}

double OtherFingerMap(const double& x){
    return 0.019 * (178 - x) / 78;
}

int FPS = 25;

bool isReal = false;

int main(){
    // Filter
    WeightedMovingFilter filter(std::vector<double>{0.4, 0.3, 0.2, 0.1}, 12);

    // HandController
    // Right Hand
    HandController::ModBusConfig modbusConfig = {
        .device = "/dev/ttyCH341USB0",
        .baudrate = 115200,
        .parity = "N",
        .dataBits = 8,
        .stopBits = 1,
        .slaveID = 3,
    };

    HandController::BasicConfig basicConfig = {
        .type = HandBase::HandType::ROHand,
        .modbusConfig = modbusConfig,
    };

    auto handCtrl = HandController::GetPtr(basicConfig);

    handCtrl->BackToInitPose();
    sleep(3);

    // DataCollector
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

    // Collect VisionPro's Data
    std::thread dataThread(&DataCollector::Run, &dataCollector);

    // HandGestureDetector
    HandGestureDetector leftHandDetector(XRBase::VisionPro);
    HandGestureDetector rightHandDetector(XRBase::VisionPro);

    HandBase::DualHandData dualHandData;
    while(true){        
        if(dataCollector.HasNewData()){
            std::cout << "Get New Data! " << std::endl;
            dualHandData.leftHandData.handPositions = dataCollector.GetLeftHandPositions();
            dualHandData.rightHandData.handPositions = dataCollector.GetRightHandPositions();
            dualHandData.leftHandData.handGesture = dataCollector.GetLeftHandGesture();
            dualHandData.rightHandData.handGesture = dataCollector.GetRightHandGesture();
            dualHandData.type = HandBase::HandType::ROHand;

            if(dualHandData.leftHandData.handPositions[0].sum() == 0 ||
               dualHandData.rightHandData.handPositions[0].sum() == 0){
                continue;
            }
        }else{
            continue;
        }

        // Check Gesture
        // For Left Hand
        bool leftIsOk = leftHandDetector.IsOkGesture(dualHandData.leftHandData);
        bool leftIsThumbsUp = leftHandDetector.IsThumbsUpGesture(dualHandData.leftHandData);
        bool leftIsPinch = leftHandDetector.IsPinchGesture(dualHandData.leftHandData);
        bool leftIsFist = leftHandDetector.IsFistGesture(dualHandData.leftHandData);
        std::cout<<"Left Hand Gesture Flag: "<<std::endl;
        std::cout<<"Is Ok: "<<leftIsOk<<std::endl;
        std::cout<<"Is Thumbs Up: "<<leftIsThumbsUp<<std::endl;
        std::cout<<"Is Pinch: "<<leftIsPinch<<std::endl;
        std::cout<<"Is Fist: "<<leftIsFist<<std::endl;

        //For Right Hand
        bool rightIsOk = rightHandDetector.IsOkGesture(dualHandData.rightHandData);
        bool rightIsThumbsUp = rightHandDetector.IsThumbsUpGesture(dualHandData.rightHandData);
        bool rightIsPinch = rightHandDetector.IsPinchGesture(dualHandData.rightHandData);
        bool rightIsFist = rightHandDetector.IsFistGesture(dualHandData.rightHandData);
        std::cout<<"Right Hand Gesture Flag: "<<std::endl;
        std::cout<<"Is Ok: "<<rightIsOk<<std::endl;
        std::cout<<"Is Thumbs Up: "<<rightIsThumbsUp<<std::endl;
        std::cout<<"Is Pinch: "<<rightIsPinch<<std::endl;
        std::cout<<"Is Fist: "<<rightIsFist<<std::endl;

        auto start = std::chrono::high_resolution_clock::now();
//        std::cout <<"Start Time: "
//                  <<std::chrono::time_point_cast<std::chrono::milliseconds>(start).time_since_epoch().count()
//                  <<std::endl;
        auto handAngle = handSolver->SolveDualHand(dualHandData);

        std::cout << "-----------------------------------------" << std::endl;
        std::cout << "HandAngle: " << std::endl;
        std::cout << handAngle << std::endl;

        // Filter the handAngle
//        filter.AddData(handAngle);
//        handAngle = filter.GetFilteredData();

        // For simulation in ros2
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

        // For real hand
        if(isReal){
            handCtrl->SetJointsAngle(handAngle.segment(0, 6));
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << " Main Loop 耗时: " << duration.count() << " ms" << std::endl;

        int framePeriod = static_cast<int>(1000.0 / FPS);
        int sleepTime = framePeriod - duration.count();

        if(sleepTime > 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
        }
    }

    handCtrl->BackToInitPose();
    sleep(3);

    // delete the thread
    dataCollector.Stop();
    dataThread.join();
}
