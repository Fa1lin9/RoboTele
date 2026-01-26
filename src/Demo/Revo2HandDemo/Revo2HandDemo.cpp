#include <DataCollector/DataCollector.hpp>
#include <HandSolver/HandSolver.hpp>
#include <HandController/HandController.hpp>
#include <Ros2Bridge/Ros2Bridge.h>
#include <HandGestureDetector/HandGestureDetector.hpp>
#include <WeightedMovingFilter/WeightedMovingFilter.hpp>

double ThumbFingerMap(const double& x){
    return x;
}

double OtherFingerMap(const double& x){
    return x * 1.155376;
}

int FPS = 25;

bool isReal = false;

int main(){
    // DataCollector
    DataCollector dataCollector("tcp://127.0.0.1:5555");

    HandSolver::BasicConfig handSolverConfig;
    handSolverConfig.handDof = 12;
    handSolverConfig.type = XRBase::XRType::VisionPro;
    auto handSolver = HandSolver::GetPtr(handSolverConfig);

    // Ros2
    Ros2Bridge leftHandBridge;
    Ros2Bridge::BasicConfig leftHandBridgeConfig;
    leftHandBridgeConfig.msgType = Ros2Bridge::MsgType::JointState;
    leftHandBridgeConfig.topicName = "/Revo2Hand/LeftHand";
    leftHandBridge.Init(leftHandBridgeConfig);

    Ros2Bridge rightHandBridge;
    Ros2Bridge::BasicConfig rightHandBridgeConfig;
    rightHandBridgeConfig.msgType = Ros2Bridge::MsgType::JointState;
    rightHandBridgeConfig.topicName = "/Revo2Hand/RightHand";
    rightHandBridge.Init(rightHandBridgeConfig);

    // Collect VisionPro's Data
    std::thread dataThread(&DataCollector::Run, &dataCollector);

    // HandGestureDetector
    HandGestureDetector gestureDetector(XRBase::VisionPro);

    HandBase::DualHandData dualHandData;
    while(true){        
        if(dataCollector.HasNewData()){
            std::cout << "Get New Data! " << std::endl;
            dualHandData.leftHandData.handPositions = dataCollector.GetLeftHandPositions();
            dualHandData.rightHandData.handPositions = dataCollector.GetRightHandPositions();
            dualHandData.leftHandData.handGesture = dataCollector.GetLeftHandGesture();
            dualHandData.rightHandData.handGesture = dataCollector.GetRightHandGesture();
            dualHandData.type = HandBase::HandType::Revo2Hand;

            if(dualHandData.leftHandData.handPositions[0].sum() == 0 ||
               dualHandData.rightHandData.handPositions[0].sum() == 0){
                continue;
            }
        }else{
            continue;
        }

        // Check Gesture
        // For Left Hand
        bool leftIsOk = gestureDetector.IsOkGesture(dualHandData.leftHandData);
        bool leftIsThumbsUp = gestureDetector.IsThumbsUpGesture(dualHandData.leftHandData);
        bool leftIsPinch = gestureDetector.IsPinchGesture(dualHandData.leftHandData);
        bool leftIsFist = gestureDetector.IsFistGesture(dualHandData.leftHandData);
        std::cout<<"Left Hand Gesture Flag: "<<std::endl;
        std::cout<<"Is Ok: "<<leftIsOk<<std::endl;
        std::cout<<"Is Thumbs Up: "<<leftIsThumbsUp<<std::endl;
        std::cout<<"Is Pinch: "<<leftIsPinch<<std::endl;
        std::cout<<"Is Fist: "<<leftIsFist<<std::endl;

        //For Right Hand
        bool rightIsOk = gestureDetector.IsOkGesture(dualHandData.rightHandData);
        bool rightIsThumbsUp = gestureDetector.IsThumbsUpGesture(dualHandData.rightHandData);
        bool rightIsPinch = gestureDetector.IsPinchGesture(dualHandData.rightHandData);
        bool rightIsFist = gestureDetector.IsFistGesture(dualHandData.rightHandData);
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

        Eigen::VectorXd leftHandAngle   = handAngle.segment(0, handAngle.size() / 2);
        Eigen::VectorXd rightHandAngle  = handAngle.segment(handAngle.size() / 2, handAngle.size() / 2);
        assert(leftHandAngle.size() == rightHandAngle.size());

        // For simulation in ros2
        // Left Hand
        humanoid_msgs::msg::JointState leftHandMsg;
        std::vector<double> leftHandPosition;
        leftHandPosition.push_back(leftHandAngle(5) * M_PI / 180.0);
        leftHandPosition.push_back(leftHandAngle(0) * M_PI / 180.0);
        leftHandPosition.push_back(leftHandAngle(0) * M_PI / 180.0);
        leftHandPosition.push_back(leftHandAngle(1) * M_PI / 180.0);
        leftHandPosition.push_back(OtherFingerMap(leftHandAngle(1)) * M_PI / 180.0);
        leftHandPosition.push_back(leftHandAngle(2) * M_PI / 180.0);
        leftHandPosition.push_back(OtherFingerMap(leftHandAngle(2)) * M_PI / 180.0);
        leftHandPosition.push_back(leftHandAngle(3) * M_PI / 180.0);
        leftHandPosition.push_back(OtherFingerMap(leftHandAngle(3)) * M_PI / 180.0);
        leftHandPosition.push_back(leftHandAngle(4) * M_PI / 180.0);
        leftHandPosition.push_back(OtherFingerMap(leftHandAngle(4)) * M_PI / 180.0);

//        leftHandPosition.push_back(leftHandAngle(0) * M_PI / 180.0);
//        leftHandPosition.push_back(leftHandAngle(5) * M_PI / 180.0);
//        leftHandPosition.push_back(leftHandAngle(1) * M_PI / 180.0);
//        leftHandPosition.push_back(leftHandAngle(2) * M_PI / 180.0);
//        leftHandPosition.push_back(leftHandAngle(3) * M_PI / 180.0);
//        leftHandPosition.push_back(leftHandAngle(4) * M_PI / 180.0);

        leftHandMsg.position() = leftHandPosition;
        leftHandMsg.name() = std::vector<std::string>(
            HandBase::Revo2HandConfig::LeftFingersName.begin(),
            HandBase::Revo2HandConfig::LeftFingersName.end()
        );

        // Right Hand
        humanoid_msgs::msg::JointState rightHandMsg;
        std::vector<double> rightHandPosition;
        rightHandPosition.push_back(rightHandAngle(5) * M_PI / 180.0);
        rightHandPosition.push_back(rightHandAngle(0) * M_PI / 180.0);
        rightHandPosition.push_back(rightHandAngle(0) * M_PI / 180.0);
        rightHandPosition.push_back(rightHandAngle(1) * M_PI / 180.0);
        rightHandPosition.push_back(OtherFingerMap(rightHandAngle(1)) * M_PI / 180.0);
        rightHandPosition.push_back(rightHandAngle(2) * M_PI / 180.0);
        rightHandPosition.push_back(OtherFingerMap(rightHandAngle(2)) * M_PI / 180.0);
        rightHandPosition.push_back(rightHandAngle(3) * M_PI / 180.0);
        rightHandPosition.push_back(OtherFingerMap(rightHandAngle(3)) * M_PI / 180.0);
        rightHandPosition.push_back(rightHandAngle(4) * M_PI / 180.0);
        rightHandPosition.push_back(OtherFingerMap(rightHandAngle(4)) * M_PI / 180.0);

//        rightHandPosition.push_back(rightHandAngle(0) * M_PI / 180.0);
//        rightHandPosition.push_back(rightHandAngle(5) * M_PI / 180.0);
//        rightHandPosition.push_back(rightHandAngle(1) * M_PI / 180.0);
//        rightHandPosition.push_back(rightHandAngle(2) * M_PI / 180.0);
//        rightHandPosition.push_back(rightHandAngle(3) * M_PI / 180.0);
//        rightHandPosition.push_back(rightHandAngle(4) * M_PI / 180.0);


        rightHandMsg.position() = rightHandPosition;
        rightHandMsg.name() = std::vector<std::string>(
            HandBase::Revo2HandConfig::RightFingersName.begin(),
            HandBase::Revo2HandConfig::RightFingersName.end()
        );

        // Send Msg
        leftHandBridge.SendMsg(leftHandMsg);
        rightHandBridge.SendMsg(rightHandMsg);

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
