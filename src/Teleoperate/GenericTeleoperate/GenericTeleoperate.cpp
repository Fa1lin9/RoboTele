#include <GenericTeleoperate/GenericTeleoperate.hpp>

bool GenericTeleoperate::Init(){
    return true;
}

GenericTeleoperate::GenericTeleoperate(const RobotTeleoperate::BasicConfig &config_)
{
    // Load Config
    this->config = config_;

    // DataCollector
    this->dataCollector.Init(this->config.address);

    std::cout << "------------------- Body Enable -------------------" << std::endl;
//    std::cout << "Head: "      << std::boolalpha << this->config.enableHead      << std::endl;
//    std::cout << "Waist: "     << std::boolalpha << this->config.enableWaist     << std::endl;
//    std::cout << "LeftArm: "   << std::boolalpha << this->config.enableLeftArm   << std::endl;
//    std::cout << "RightArm: "  << std::boolalpha << this->config.enableRightArm  << std::endl;
//    std::cout << "LeftLeg: "   << std::boolalpha << this->config.enableLeftLeg   << std::endl;
//    std::cout << "RightLeg: "  << std::boolalpha << this->config.enableRightLeg  << std::endl;

    std::cout << "Head: "      << this->config.enableHead      << std::endl;
    std::cout << "Waist: "     << this->config.enableWaist     << std::endl;
    std::cout << "LeftArm: "   << this->config.enableLeftArm   << std::endl;
    std::cout << "RightArm: "  << this->config.enableRightArm  << std::endl;
    std::cout << "LeftLeg: "   << this->config.enableLeftLeg   << std::endl;
    std::cout << "RightLeg: "  << this->config.enableRightLeg  << std::endl;
    std::cout << "------------------- Body Enable -------------------" << std::endl;


    // For Head and Waist Solver
    if(this->config.enableHead){
        this->headSolver.Init(this->config.headSolverConfigPath);
        this->headJointsInfo = this->headSolver.GetJointsInfo();
    }

    if(this->config.enableWaist){
        this->waistSolver.Init(this->config.waistSolverConfigPath);
        waistJointsInfo = this->waistSolver.GetJointsInfo();
    }

    // Ptr
    std::cout << "------------------- Configuration Path -------------------" << std::endl;
    std::cout << "ArmSolver: \n" << this->config.armSolverConfigPath << std::endl;
    std::cout << "HeadSolver: \n" << this->config.headSolverConfigPath << std::endl;
    std::cout << "WaistSolver: \n" << this->config.waistSolverConfigPath << std::endl;
    std::cout << "Transform: \n" << this->config.transformConfigPath << std::endl;
    std::cout << "RobotHardware: \n" << this->config.hardwareConfigPath << std::endl;
    std::cout << "------------------- Configuration Path -------------------" << std::endl;

    this->armSolverPtr = ArmSolver::GetPtr(this->config.armSolverConfigPath);

    this->transformPtr = Transform::GetPtr(this->config.transformConfigPath);

    if(this->config.isReal){
        std::cout << "------------------- Use RobotHardware -------------------" << std::endl;
        this->hardwarePtr = RobotHardware::GetPtr(this->config.hardwareConfigPath);
    }

    this->ros2Bridge.Init(this->config.bridgeConfig);

    this->handGestureDectector.Init(this->config.xrType);

    // Set qLast
    std::cout << "Current Robot's TotalDof: " << this->armSolverPtr->GetTotalDof() << std::endl;
    this->qLast = Eigen::VectorXd::Zero(this->armSolverPtr->GetTotalDof());

    // Speed Limits
    double threshold = this->config.FPS * M_PI / 180.0;
    this->speedThreshold = Eigen::VectorXd::Constant(this->armSolverPtr->GetTotalDof(), threshold);

}

GenericTeleoperate::~GenericTeleoperate(){

}

bool GenericTeleoperate::StartTeleop(bool verbose){
    // Filter
    WeightedMovingFilter filter(this->config.filterWeight, this->armSolverPtr->GetTotalDof());

    this->startFlag = true;
    this->saveFlag = true;

    // Collector VisionPro's Data
    std::thread dataThread(&DataCollector::Run, &dataCollector);

    // Some Valuables
    Transform::MsgConfig msgConfig;
    Eigen::VectorXd qEigen;
    boost::optional<Eigen::VectorXd> q;

    // Flag
    bool isStart = false;
    bool isEnd = false;
    bool isFirstCheck = false;
    while(this->startFlag){
        auto start = std::chrono::high_resolution_clock::now();

        // Get Data from XR Device
        if(this->dataCollector.HasNewData()){
//            std::cout << "Get New Data! " << std::endl;
            this->poseMatrix = this->dataCollector.GetPoseMatrix();

            this->dualHandData.leftHandData.handPositions = this->dataCollector.GetLeftHandPositions();
            this->dualHandData.rightHandData.handPositions = this->dataCollector.GetRightHandPositions();

            this->dualHandData.leftHandData.handGesture = this->dataCollector.GetLeftHandGesture();
            this->dualHandData.leftHandData.handGesture = this->dataCollector.GetRightHandGesture();

            this->dualHandData.type = HandBase::HandType::ROHand;
        }else{
            continue;
        }

        // Check data validity
        this->CheckDataValid();

        if(verbose){
            std::cout << "------------- Original Data -------------" << std::endl;
            std::cout << "Head Pose:\n" << this->poseMatrix[0] << std::endl;
            std::cout << "Left Wrist Pose:\n" << this->poseMatrix[1] << std::endl;
            std::cout << "Right Wrist Pose:\n" << this->poseMatrix[2] << std::endl;
            std::cout << "-----------------------------------------" << std::endl;
        }

        // Start or End the Teleoperation according to the Hand Gesture
        if(!isStart){
            isStart =
                    this->handGestureDectector.IsOkGesture(this->dualHandData.leftHandData) &&
                    this->handGestureDectector.IsOkGesture(this->dualHandData.rightHandData);
            if(!isStart){
                continue;
            }
            // Set initial head pose
            msgConfig.initHeadPose = this->poseMatrix[0];
            std::cout << "Initial Head Pose: " << std::endl;
            std::cout << msgConfig.initHeadPose << std::endl;
            std::cout << "[GenericTeleoperate] The handGesture is OK, now start the teleoperation! "<<std::endl;
        }

        if(!isEnd){
            isEnd =
                    this->handGestureDectector.IsThumbsUpGesture(this->dualHandData.leftHandData) ||
                    this->handGestureDectector.IsThumbsUpGesture(this->dualHandData.rightHandData);
        }else{
            std::cout << "[GenericTeleoperate] The handGesture is thumbs up, now end the teleoperation! "<<std::endl;
            break;
        }

        // Message Config
        msgConfig.head2XRWorldPose = this->poseMatrix[0];
        msgConfig.leftWrist2XRWorldPose = this->poseMatrix[1];
        msgConfig.rightWrist2XRWorldPose = this->poseMatrix[2];

        bool modeFlag =
                this->handGestureDectector.IsFistGesture(this->dualHandData.leftHandData) &&
                this->handGestureDectector.IsFistGesture(this->dualHandData.rightHandData);
        if(modeFlag)
        {
            msgConfig.mode = Transform::TeleMode::WaistMode;
            msgConfig.modeHeadPose = this->poseMatrix[0];
        }else{
            msgConfig.mode = Transform::TeleMode::HeadMode;
        }

        // Transformed the Matrix
        std::vector<Eigen::Matrix4d> transformedMsg = this->transformPtr->Solve(msgConfig);

        if(verbose){
            std::cout << "------------- Transformed Data -------------" << std::endl;
            std::cout << "Transformed Left Wrist Pose:\n" << transformedMsg[0] << std::endl;
            std::cout << "Transformed Right Wrist Pose:\n" << transformedMsg[1] << std::endl;
            std::cout << "Transformed Head Robot World Pose:\n" << transformedMsg[2] << std::endl;
            std::cout << "Upper-Body Robot World Pose:\n" << transformedMsg[3] << std::endl;
            std::cout << "--------------------------------------------" << std::endl;
        }

        // Use ArmSolver to Solve
        std::cout<<"-------------- Start to solve --------------"<<std::endl;

        q = armSolverPtr->Solve({transformedMsg[0],transformedMsg[1]},
                                this->qLast,
                                false);

        // Check the Solution
        if(q.has_value()){
            qEigen = q.value();

            if (msgConfig.mode == Transform::TeleMode::HeadMode){
                // Control the Head
                headRPY = this->headSolver.Solve(transformedMsg[3].inverse() * transformedMsg[2]);
                std::cout<<"HeadRPY: "<<headRPY<<std::endl;
            }else if (msgConfig.mode == Transform::TeleMode::WaistMode)
            {
                // Control the Waist
                waistRPY = this->waistSolver.Solve(transformedMsg[2]);
                std::cout<<"WaistRPY: "<<waistRPY<<std::endl;
            }
            if(this->config.enableHead){
                // Set Value to Head
//                qEigen(headJointsInfo[0].index) = headRPY(2); // Yaw
//                qEigen(headJointsInfo[1].index) = - headRPY(1); // Pitch
//                qEigen(headJointsInfo[2].index) = headRPY(0); // Row
                for(size_t i=0;i<3;i++){
                    if(headJointsInfo[i].index != -1){
                        qEigen(headJointsInfo[i].index) =
                                headRPY(headJointsInfo[i].type) * headJointsInfo[i].direction;
                    }
                }
            }

            if(this->config.enableWaist){
                // Set Value to Waist
//                qEigen(waistJointsInfo[0].index) = waistRPY(0); // Row
//                qEigen(waistJointsInfo[1].index) = waistRPY(2); // Yaw
//                qEigen(waistJointsInfo[2].index) = -waistRPY(1); // Pitch
                for(size_t i=0;i<3;i++){
                    if(waistJointsInfo[i].index != -1){
                        qEigen(waistJointsInfo[i].index) =
                                waistRPY(waistJointsInfo[i].type) * waistJointsInfo[i].direction;
                    }
                }
            }

            // Check solution
            if(this->config.isCheckSolution){
                if(!isFirstCheck){
                    isFirstCheck = true;
                }else{
                    if(!this->CheckSolutionValid(qEigen, this->qLast)){
                        qEigen = this->qLast;
                    }
                }
            }

            // Filter the Eigen
            if(this->config.isFilterSolution){
                filter.AddData(qEigen);
                qEigen = filter.GetFilteredData();
            }

            if(this->config.isSim){
                // send to ros2
                ti5_interfaces::msg::JointStateWithoutStamp msg;
                std::vector<double> qVec(qEigen.data(), qEigen.data() + qEigen.size());
                msg.position() = qVec;
                msg.name() = this->armSolverPtr->GetJointNames();

                this->ros2Bridge.SendMsg(msg);
            }

            // Check the angle of the dual-arm
//            std::vector<double> qTargetLeftArm(this->armSolverPtr->GetLeftArmQIndex().size());
//            std::vector<double> qTargetRightArm(this->armSolverPtr->GetRightArmQIndex().size());
//            auto leftArmQIndex = this->armSolverPtr->GetLeftArmQIndex();
//            auto rightArmQIndex = this->armSolverPtr->GetRightArmQIndex();

//            std::cout << "qTargetLeftArm:\n";
//            for(size_t i = 0; i < leftArmQIndex.size(); i++){
//                qTargetLeftArm[i] = qEigen(leftArmQIndex[i]);
//                std::cout << "Joint[" << i << "] (qIndex=" << leftArmQIndex[i]
//                          << ") = " << qTargetLeftArm[i] << "\n";
//            }

//            std::cout << "qTargetRightArm:\n";
//            for(size_t i = 0; i < rightArmQIndex.size(); i++){
//                qTargetRightArm[i] = qEigen(rightArmQIndex[i]);
//                std::cout << "Joint[" << i << "] (qIndex=" << rightArmQIndex[i]
//                          << ") = " << qTargetRightArm[i] << "\n";
//            }

            if(this->config.isReal){
                // TODO
                std::vector<double> qTargetLeftArm(this->armSolverPtr->GetLeftArmQIndex().size());
                std::vector<double> qTargetRightArm(this->armSolverPtr->GetRightArmQIndex().size());
                auto leftArmQIndex = this->armSolverPtr->GetLeftArmQIndex();
                auto rightArmQIndex = this->armSolverPtr->GetRightArmQIndex();

                std::cout << "qTargetLeftArm:\n";
                for(size_t i = 0; i < leftArmQIndex.size(); i++){
                    qTargetLeftArm[i] = qEigen(leftArmQIndex[i]);
                    std::cout << "Joint[" << i << "] (qIndex=" << leftArmQIndex[i]
                              << ") = " << qTargetLeftArm[i] << "\n";
                }

                std::cout << "qTargetRightArm:\n";
                for(size_t i = 0; i < rightArmQIndex.size(); i++){
                    qTargetRightArm[i] = qEigen(rightArmQIndex[i]);
                    std::cout << "Joint[" << i << "] (qIndex=" << rightArmQIndex[i]
                              << ") = " << qTargetRightArm[i] << "\n";
                }

                RobotHardware::HumanoidCmd cmd = {
                    .enableHead = false,
                    .enableLeftArm = true,
                    .enableRightArm = true,
                    .enableWaist = false,
                    .enableLeftLeg = false,
                    .enableRightLeg = false,
                    .verbose = true,
                    .qTargetLeftArm = qTargetLeftArm,
                    .qTargetRightArm = qTargetRightArm,
//                    .qTargetLeftArm = std::vector<double>{0., 0, 0, 0.5, 0},
//                    .qTargetRightArm = std::vector<double>{0., 0, 0, 0.5, 0},
                };

//                auto state = this->hardwarePtr->GetState(true);
                this->hardwarePtr->SendCmd(cmd);

            }

            // set the initial value of the joint
            if(this->config.isReal){
            // For Real Robot
//                qLast = this->hardwarePtr->GetJointsAngleEigen();
            }else{
                this->qLast = qEigen;
            }

            std::cout << "q:\n" << std::fixed << std::setprecision(5) << qEigen << std::endl;
        }else{
            std::cout<<" Solve failed! "<<std::endl;
            continue;
        }

        // Save Data to Log
        if(this->saveFlag){
            this->csvWriter.WriteEigenVector(q.value());
        }
        std::cout<<"---------------- Solver over ----------------"<<std::endl;

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << " Main Loop 耗时: " << duration.count() << " ms" << std::endl;

        int framePeriod = static_cast<int>(1000.0 / this->config.FPS);
        int sleepTime = framePeriod - duration.count();

        if(sleepTime > 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
        }
    }

    // delete the thread
    this->dataCollector.Stop();
    dataThread.join();

    return true;
}

bool GenericTeleoperate::StopTeleop(){
    if(!this->startFlag){
        std::cout<<"Teleoperation has ended! "<<std::endl;
    }else{
        this->startFlag = false;
    }

    return true;
}

void GenericTeleoperate::Info()
{

}

bool GenericTeleoperate::CheckDataValid(){
    // PoseMatrix
    if (this->poseMatrix[0].isZero(1e-9) ||
        this->poseMatrix[1].isZero(1e-9) ||
        this->poseMatrix[2].isZero(1e-9)){
        std::cerr << "[GenericTeleoperate::CheckData] One of poseMatrix is zero!" << std::endl;
        return false;
    }

    if (this->poseMatrix.empty()) {
        std::cerr << "[GenericTeleoperate::CheckData] poseMatrix is empty!" << std::endl;
        return false;
    }

    // ensure poseMatrix has at least 3 matrices
    if (this->poseMatrix.size() < 3) {
        std::cerr << "[GenericTeleoperate::CheckData] poseMatrix size < 3!" << std::endl;
        return false;
    }

    // HandData
    const auto& left  = this->dualHandData.leftHandData.handPositions;
    const auto& right = this->dualHandData.rightHandData.handPositions;

    if (left.empty() || right.empty()) {
        std::cerr << "[GenericTeleoperate::CheckData] HandData is empty!" << std::endl;
        return false;
    }

    if (left[0].isZero(1e-9) || right[0].isZero(1e-9)) {
        std::cerr << "[GenericTeleoperate::CheckData] First hand position is zero!" << std::endl;
        return false;
    }

    return true;
}

bool GenericTeleoperate::CheckSolutionValid(const Eigen::VectorXd& sol,
                                            const Eigen::VectorXd& qLast)
{
    Eigen::VectorXd diff = sol - qLast;

    bool outOfBounds = false;
    for(size_t i=0;i<diff.size();i++){
        if(std::abs(diff(i)) > this->speedThreshold[i]){
            outOfBounds = true;
            break;
        }
    }

    if(outOfBounds){
        std::cout<<"[GenericTeleoperate::CheckSolutionValid] The solution is invalid! "<<std::endl;
        return false;
    }else{
        return true;
    }
}
