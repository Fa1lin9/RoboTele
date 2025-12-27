#include <Ti5RobotTeleoperate/Ti5RobotTeleoperate.hpp>

bool Ti5RobotTeleoperate::Init(){
    return true;
}

Ti5RobotTeleoperate::Ti5RobotTeleoperate(const RobotTeleoperate::BasicConfig &config)
//    :address(config.address),
//     dataCollector(VisionProCollector(this->address)),
//     isSim(config.isSim),
//     isReal(config.isReal),
//     FPS(config.FPS)
{
    this->address = config.address;
    this->dataCollector.Init(this->address);
    this->isSim = config.isSim;
    this->isReal = config.isReal;
    this->isCheckSolution = config.isCheckSolution;
    this->isFilterSolution = config.isFilterSolution;
    this->FPS = config.FPS;
    this->filterWeight = config.filterWeight;

    this->useHead = config.useHead;
    this->useWaist = config.useWaist;

    // For Head and Waist Solver
    if(this->useHead){
        this->headSolver.Init(config.robotType);
        this->headJointsInfo = this->headSolver.GetJointsInfo();
    }

    if(this->useWaist){
        this->waistSolver.Init(config.robotType);
        waistJointsInfo = this->waistSolver.GetJointsInfo();
    }

    this->ikSolverPtr = ArmSolver::GetPtr(config.solverConfig);

    this->transformPtr = Transform::GetPtr(config.transformConfig);

    this->physicalRobotPtr = PhysicalRobot::GetPtr(config.robotConfig);

    this->ros2Bridge.Init(config.bridgeConfig);

    this->handGestureDectector.Init(config.xrType);

    // qLast
    this->qLast = Eigen::VectorXd::Zero(this->ikSolverPtr->GetDofTotal());

    // Initial Pose For Real Robot
//    this->qLast.segment(4,7) << -0.72, -1.0, 0.57, -1.0, 0.83, 0, 0;
//    this->qLast.segment(14,7) << 0.72, 1.0, -0.57, 1.0, -0.83, 0, 0;

    this->qLast.segment(4,7) << -0.72, -1.0, -1.0, -1.0, 0.83, 0, 0;
    this->qLast.segment(14,7) << 0.72, 1.0, 1.0, 1.0, -0.83, 0, 0;

    // Speed Limits
    double threshold = this->FPS * M_PI / 180.0;
    this->speedThreshold = Eigen::VectorXd::Constant(this->ikSolverPtr->GetDofTotal(), threshold);

}

Ti5RobotTeleoperate::~Ti5RobotTeleoperate(){

}

bool Ti5RobotTeleoperate::StartTeleop(bool verbose){
    // Filter
//    WeightedMovingFilter filter(std::vector<double>{0.4, 0.3, 0.2, 0.1}, this->ikSolverPtr->GetDofTotal());
    WeightedMovingFilter filter(this->filterWeight, this->ikSolverPtr->GetDofTotal());

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
            std::cout << "[Ti5RobotTeleoperate] The handGesture is OK, now start the teleoperation! "<<std::endl;
        }

        if(!isEnd){
            isEnd =
                    this->handGestureDectector.IsThumbsUpGesture(this->dualHandData.leftHandData) ||
                    this->handGestureDectector.IsThumbsUpGesture(this->dualHandData.rightHandData);
        }else{
            std::cout << "[Ti5RobotTeleoperate] The handGesture is thumbs up, now end the teleoperation! "<<std::endl;
            break;
        }

        // Message Config
        // Pose Matrix
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

        q = ikSolverPtr->Solve({transformedMsg[0],transformedMsg[1]},
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
            if(this->useHead){
                // Set Value to Head
                qEigen(headJointsInfo[0].index) = headRPY(2); // Yaw
                qEigen(headJointsInfo[1].index) = - headRPY(1); // Pitch
                qEigen(headJointsInfo[2].index) = headRPY(0); // Row
            }

            if(this->useWaist){
                // Set Value to Waist
                qEigen(waistJointsInfo[0].index) = waistRPY(0); // Row
                qEigen(waistJointsInfo[1].index) = waistRPY(2); // Yaw
                qEigen(waistJointsInfo[2].index) = -waistRPY(1); // Pitch
            }

            // Check solution
            if(this->isCheckSolution){
                if(!isFirstCheck){
                    isFirstCheck = true;
                }else{
                    if(!this->CheckSolutionValid(qEigen, this->qLast)){
                        qEigen = this->qLast;
                    }
                }
            }

            // Filter the Eigen
            if(this->isFilterSolution){
                filter.AddData(qEigen);
                qEigen = filter.GetFilteredData();
            }

            if(this->isSim){
                // send to ros2
                ti5_interfaces::msg::JointStateWithoutStamp msg;
                std::vector<double> qVec(qEigen.data(), qEigen.data() + qEigen.size());
                msg.position() = qVec;
                msg.name() = this->ikSolverPtr->GetJointNames();
//                for(size_t i=0;i<msg.name().size();i++){
//                    std::cout<<"Msg Joint "<<i<<": "<<msg.name()[i]<<std::endl;
//                }
                // Temp send 14 joints value just to apply to GunmpGan's current version
//                std::vector<double> qVecOnlyArm;
//                qVecOnlyArm.reserve(14);
//                qVecOnlyArm.insert(qVecOnlyArm.end(),qVec.begin() + 4, qVec.begin() + 4 + 7);
//                qVecOnlyArm.insert(qVecOnlyArm.end(),qVec.end() - 7, qVec.end());
//                msg.position() = qVecOnlyArm;

    //            std::cout<<"The size of the postion of the msg is "<<msg.position().size()<<std::endl;
    //            for(size_t i=0;i<msg.position().size();i++){
    //                std::cout<<"SendMsg: Joint "<<i<<" Value: "<<msg.position()[i]<<std::endl;
    //            }
                this->ros2Bridge.SendMsg(msg);
            }

            if(this->isReal){
                // TODO

            }

            // set the initial value of the joint
            // For Simulated Robot
            this->qLast = qEigen;

            // For Real Robot
//            qLast = physicalRobotPtr->GetJointsAngleEigen();

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

        int framePeriod = static_cast<int>(1000.0 / this->FPS);
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

bool Ti5RobotTeleoperate::StopTeleop(){
    if(!this->startFlag){
        std::cout<<"Teleoperation has ended! "<<std::endl;
    }else{
        this->startFlag = false;
    }

    return true;
}

bool Ti5RobotTeleoperate::CheckDataValid(){
    // PoseMatrix
    if (this->poseMatrix[0].isZero(1e-9) ||
        this->poseMatrix[1].isZero(1e-9) ||
        this->poseMatrix[2].isZero(1e-9)){
        std::cerr << "[Ti5RobotTeleoperate::CheckData] One of poseMatrix is zero!" << std::endl;
        return false;
    }

    if (this->poseMatrix.empty()) {
        std::cerr << "[Ti5RobotTeleoperate::CheckData] poseMatrix is empty!" << std::endl;
        return false;
    }

    // ensure poseMatrix has at least 3 matrices
    if (this->poseMatrix.size() < 3) {
        std::cerr << "[Ti5RobotTeleoperate::CheckData] poseMatrix size < 3!" << std::endl;
        return false;
    }

    // HandData
    const auto& left  = this->dualHandData.leftHandData.handPositions;
    const auto& right = this->dualHandData.rightHandData.handPositions;

    if (left.empty() || right.empty()) {
        std::cerr << "[Ti5RobotTeleoperate::CheckData] HandData is empty!" << std::endl;
        return false;
    }

    if (left[0].isZero(1e-9) || right[0].isZero(1e-9)) {
        std::cerr << "[Ti5RobotTeleoperate::CheckData] First hand position is zero!" << std::endl;
        return false;
    }

    return true;
}

bool Ti5RobotTeleoperate::CheckSolutionValid(const Eigen::VectorXd& sol,
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
        std::cout<<"[Ti5RobotTeleoperate::CheckSolutionValid] The solution is invalid! "<<std::endl;
        return false;
    }else{
        return true;
    }
}
