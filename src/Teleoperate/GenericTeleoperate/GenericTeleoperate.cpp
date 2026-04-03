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
    std::cout << "LeftHand: "   << this->config.enableLeftHand   << std::endl;
    std::cout << "RightHand: "  << this->config.enableRightHand  << std::endl;
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

    // For HandSolver
    if(this->config.enableLeftHand || this->config.enableRightHand){
        HandSolver::BasicConfig handSolverConfig = {
            .type = this->config.xrType,
            .handDof = 12,
        };
        this->handSolverPtr = HandSolver::GetPtr(handSolverConfig);
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

    // Ros2Bridge
    // Body
    this->bodyBridge.Init(this->config.bodyBridgeConfig);

    // For Left and Right Hand
    if(this->config.enableLeftHand){
        this->leftHandBridge.Init(this->config.leftHandBridgeConfig);
    }
    if(this->config.enableRightHand){
        this->rightHandBridge.Init(this->config.rightHandBridgeConfig);
    }

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
    Eigen::VectorXd handAngle;
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

            this->dualHandData.type = this->config.handType;
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
        if(modeFlag && this->config.enableWaist)
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

        auto optimStart = std::chrono::high_resolution_clock::now();
        q = this->armSolverPtr->Solve({transformedMsg[0],transformedMsg[1]},
                                this->qLast,
                                true);

        auto optimEnd = std::chrono::high_resolution_clock::now();
        auto optimEuration =
                std::chrono::duration_cast<std::chrono::milliseconds>(optimEnd - optimStart);

        // Solve Hand
        handAngle = this->handSolverPtr->SolveDualHand(this->dualHandData);

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
                for(size_t i=0;i<3;i++){
                    if(headJointsInfo[i].index != -1){
                        qEigen(headJointsInfo[i].index) =
                                headRPY(headJointsInfo[i].type) * headJointsInfo[i].direction;
                    }
                }
            }

            if(this->config.enableWaist){
                // Set Value to Waist
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
                // Send to Ros2
                // Body
                humanoid_msgs::msg::JointState bodyMsg;
                std::vector<double> qVec(qEigen.data(), qEigen.data() + qEigen.size());
                bodyMsg.position() = qVec;
                bodyMsg.name() = this->armSolverPtr->GetJointNames();

                this->bodyBridge.SendMsg(bodyMsg);

                int total = handAngle.size();
                int half  = total / 2;

                Eigen::VectorXd leftSeg  = handAngle.segment(0, half);
                Eigen::VectorXd rightSeg = handAngle.segment(half, half);

                auto fingerNames = this->handSolverPtr->GetFingersName();


                // =============== Left Hand ===============
                if (this->config.enableLeftHand) {
                    humanoid_msgs::msg::JointState leftHandMsg;

                    leftHandMsg.name() = fingerNames;

                    leftHandMsg.position().assign(leftSeg.data(), leftSeg.data() + half);

                    this->leftHandBridge.SendMsg(leftHandMsg);
                }


                // =============== Right Hand ===============
                if (this->config.enableRightHand) {
                    humanoid_msgs::msg::JointState rightHandMsg;

                    rightHandMsg.name() = fingerNames;

                    rightHandMsg.position().assign(rightSeg.data(), rightSeg.data() + half);

                    this->rightHandBridge.SendMsg(rightHandMsg);
                }

            }

            if(this->config.isReal){
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
            // Write the solved joint angle
//            this->csvWriter.WriteEigenVector(q.value());

            auto temp = this->armSolverPtr->Forward(q.value());

            // 平移
            Eigen::Vector3d transLeftRobot  = temp[0].translation();
            Eigen::Vector3d transRightRobot = temp[1].translation();
            Eigen::Vector3d transLeftHuman  = transformedMsg[0].block<3,1>(0,3);
            Eigen::Vector3d transRightHuman = transformedMsg[1].block<3,1>(0,3);

            auto SafeEulerAnglesZYX = [](const Eigen::Matrix3d& R) -> Eigen::Vector3d
            {
                // ZYX: yaw(Z), pitch(Y), roll(X)
                Eigen::Vector3d euler = R.eulerAngles(2,1,0);

                double yaw   = euler[0];
                double pitch = euler[1];
                double roll  = euler[2];

                // 👉 ZYX 的奇异点仍然在 pitch = ±pi/2（万向锁）
                // 不建议像之前那样强行 ±pi 修正（会引入跳变）
                // 这里只做范围规范化

                auto NormalizeAngle = [](double a)
                {
                    if(a > M_PI)       a -= 2.0 * M_PI;
                    else if(a < -M_PI) a += 2.0 * M_PI;
                    return a;
                };

                yaw   = NormalizeAngle(yaw);
                pitch = NormalizeAngle(pitch);
                roll  = NormalizeAngle(roll);

                // ⚠️ 返回顺序：roll, pitch, yaw（和你之前保持一致接口）
                return Eigen::Vector3d(roll, pitch, yaw);
            };

            // 提取欧拉角
            Eigen::Vector3d eulerLeftRobot  = SafeEulerAnglesZYX(temp[0].rotation());
            Eigen::Vector3d eulerRightRobot = SafeEulerAnglesZYX(temp[1].rotation());
            Eigen::Vector3d eulerLeftHuman  = SafeEulerAnglesZYX(transformedMsg[0].block<3,3>(0,0));
            Eigen::Vector3d eulerRightHuman = SafeEulerAnglesZYX(transformedMsg[1].block<3,3>(0,0));

            // 拼接：12平移 + 12欧拉角 + 1时间 = 25维
            Eigen::VectorXd all(25);
            all << transLeftRobot,
                   transLeftHuman,
                   transRightRobot,
                   transRightHuman,
                   eulerLeftRobot,
                   eulerLeftHuman,
                   eulerRightRobot,
                   eulerRightHuman,
                   static_cast<double>(optimEuration.count());

            // 写 CSV
            this->csvWriter.WriteEigenVector(all);

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
