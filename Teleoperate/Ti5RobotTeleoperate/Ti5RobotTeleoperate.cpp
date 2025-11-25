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
    this->FPS = config.FPS;

    // qInit
    this->qInit = Eigen::VectorXd::Zero(21);
    qInit.segment(4,7) << -0.72, -1.0, 0.57, -1.0, 0.83, 0, 0;
    qInit.segment(14,7) << 0.72, 1.0, -0.57, 1.0, -0.83, 0, 0;

    this->ikSolverPtr = DualArmSolver::GetPtr(config.solverConfig);

    this->transformPtr = CoordinateTransform::GetPtr(config.transformConfig);

    this->physicalRobotPtr = PhysicalRobot::GetPtr(config.robotConfig);

    this->ros2Bridge.Init(config.bridgeConfig);
}

//Ti5RobotTeleoperate::Ti5RobotTeleoperate(const std::string &filePath)
//{
//    // qInit
//    this->qInit = Eigen::VectorXd::Zero(21);
//    qInit.segment(4,7) << -0.72, -1.0, 0.57, -1.0, 0.83, 0, 0;
//    qInit.segment(14,7) << 0.72, 1.0, -0.57, 1.0, -0.83, 0, 0;

//    JsonParser jsonParser(filePath);
//    json::object rootObj = jsonParser.GetJsonObject();

//    json::object solverObj = rootObj["SolverConfig"].as_object();
//    json::object transformObj = rootObj["TransformConfig"].as_object();
//    json::object physicalRObotObj = rootObj["PhysicalRobotConfig"].as_object();
//    json::object ros2BridgeObj = rootObj["Ros2BridgeConfig"].as_object();

//    RobotType::Type robotType = RobotType::GetTypeFromStr(rootObj["RobotType"].as_string().c_str());

//    // IKSolver
//    IKSolver::BasicConfig solverConfig = {
////        .type = IKSolver::Type::Ti5Robot,
//        .robotType = robotType,
////        .baseFrameName = {"BASE_S"},
//        .baseFrameName = JsonParser::JsonArray2StdVecStr(solverObj["BaseFrameName"].as_array()),
////        .targetFrameName = {"L_WRIST_R", "R_WRIST_R"},
//        .targetFrameName = JsonParser::JsonArray2StdVecStr(solverObj["TargetFrameName"].as_array()),
//        .baseOffset = {JsonParser::JsonArray2EigenMatrixXd(solverObj["BaseOffset"].as_array()[0].as_array())},
//        // for nlopt
////        .maxIteration = 400,
////        .relativeTol = 1e-2,
//        // for ipopt
//        .maxIteration = static_cast<int>(solverObj["MaxIteration"].as_int64()),
//        .relativeTol = solverObj["RelativeTol"].as_double(),
//        .dofLeftArm = static_cast<int>(solverObj["DofLeftArm"].as_int64()),
//        .dofRightArm = static_cast<int>(solverObj["DofRightArm"].as_int64()),
//    };

//    // CoordinateTransform
//    CoordinateTransform::BasicConfig transformConfig = {
//        .T_Head2Waist = JsonParser::JsonArray2EigenMatrixXd(transformObj["T_Head2Waist"].as_array()),
//        .T_XR2Robot = JsonParser::JsonArray2EigenMatrixXd(transformObj["T_XR2Robot"].as_array()),
//        .T_Robot2LeftWrist = JsonParser::JsonArray2EigenMatrixXd(transformObj["T_Robot2LeftWrist"].as_array()),
//        .T_Robot2RightWrist = JsonParser::JsonArray2EigenMatrixXd(transformObj["T_Robot2RightWrist"].as_array()),
//        .offset = JsonParser::JsonArray2EigenVectorXd(transformObj["Offset"].as_array()),
//        .type = CoordinateTransform::GetTypeFromStr(transformObj["Type"].as_string().c_str()),
//    };

//    // PhysicalRobot
//    PhysicalRobot::BasicConfig physicalRobotConfig = {
//        .robotType = robotType,
//    };

//    // Ros2Bridge
//    Ros2Bridge::BasicConfig bridgeConfig = {
//        .topicName = ros2BridgeObj["TopicName"].as_string().c_str(),
//        .msgType = Ros2Bridge::GetMsgTypeFromStr(ros2BridgeObj["MsgType"].as_string().c_str()),
//    };

//    this->address = rootObj["Address"].as_string().c_str();
//    this->dataCollector.Init(this->address);
//    this->FPS = rootObj["FPS"].as_int64();
//    this->isSim = rootObj["IsSimulation"].as_bool();
//    this->isReal = rootObj["IsReal"].as_bool();

//    this->ikSolverPtr = IKSolver::GetPtr(solverConfig);

//    this->transformPtr = CoordinateTransform::GetPtr(transformConfig);

//    this->physicalRobotPtr = PhysicalRobot::GetPtr(physicalRobotConfig);

//    this->ros2Bridge.Init(bridgeConfig);
//}

Ti5RobotTeleoperate::~Ti5RobotTeleoperate(){

}

bool Ti5RobotTeleoperate::StartTeleoperate(bool verbose){
    // Filter
    WeightedMovingFilter filter(std::vector<double>{0.4, 0.3, 0.2, 0.1}, this->ikSolverPtr->GetDofTotal());

//    int FPS = 25;
    this->pauseFlag = false;
    this->startFlag = true;
    this->saveFlag = false;

//    // Collector VisionPro's Data
//    std::thread dataThread([this](){
//        this->dataCollector.run();
//    });

    while(this->startFlag){
        if(this->pauseFlag){
            std::cout<<"Teleoperation Stop ! "<<std::endl;
            sleep(1);
            continue;
        }

        auto start = std::chrono::high_resolution_clock::now();
        std::vector<Eigen::Matrix4d> msg = this->dataCollector.GetValue();

        if(msg.size()==0 || msg.empty()){
            std::string error = " The message received is not qualified !";
            throw std::invalid_argument(error);
            continue;
        }

        if(verbose){
            std::cout << "--------------------------" << std::endl;
            std::cout << "Head Pose:\n" << msg[0] << std::endl;
            std::cout << "Left Wrist Pose:\n" << msg[1] << std::endl;
            std::cout << "Right Wrist Pose:\n" << msg[2] << std::endl;
            std::cout << "--------------------------" << std::endl;
        }

        CoordinateTransform::MsgConfig msgConfig{
            .head2xrWorldPose = msg[0],
            .leftWrist2xrWorldPose = msg[1],
            .rightWrist2xrWorldPose = msg[2],
        };

        std::vector<Eigen::Matrix4d> transformedMsg = this->transformPtr->Transform(msgConfig);

        if(verbose){
            std::cout << "--------------------------" << std::endl;
            std::cout << "Transformed Left Wrist Pose:\n" << transformedMsg[0] << std::endl;
            std::cout << "Transformed Right Wrist Pose:\n" << transformedMsg[1] << std::endl;
            std::cout << "--------------------------" << std::endl;
        }

        std::cout<<"-------------- Start to solve --------------"<<std::endl;

//        auto solveStart = std::chrono::high_resolution_clock::now();
        boost::optional<Eigen::VectorXd> q = ikSolverPtr->Solve(transformedMsg, qInit, false);
//        auto solveEnd = std::chrono::high_resolution_clock::now();
//        auto solveDuration = std::chrono::duration_cast<std::chrono::milliseconds>(solveEnd - solveStart);
//        std::cout << " Solve 耗时: " << solveDuration.count() << " ms" << std::endl;

        Eigen::VectorXd qEigen;

        if(q.has_value()){
            qEigen = q.value();

            // Filter
            filter.AddData(qEigen);
            qEigen = filter.GetFilteredData();

            if(this->isSim){
                // send to ros2
                ti5_interfaces::msg::JointStateWithoutStamp msg;
                std::vector<double> qVec(qEigen.data(), qEigen.data() + qEigen.size());
    //            msg.position() = qVec;
                // Temp send 14 joints value just to apply to GunmpGan's current version
                std::vector<double> qVecOnlyArm;
                qVecOnlyArm.reserve(14);
                qVecOnlyArm.insert(qVecOnlyArm.end(),qVec.begin() + 4, qVec.begin() + 4 + 7);
                qVecOnlyArm.insert(qVecOnlyArm.end(),qVec.end() - 7, qVec.end());
                msg.position() = qVecOnlyArm;

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
            qInit = qEigen;

//            qInit = physicalRobotPtr->GetJointsAngleEigen();
            std::cout << "q:\n" << std::fixed << std::setprecision(5) << q << std::endl;
        }else{
            std::cout<<" Solve failed! "<<std::endl;
            continue;
        }

        // save data to log
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

//    // delete the thread
//    dataThread.join();

    return true;
}

bool Ti5RobotTeleoperate::PauseTeleoperate(){
    this->pauseFlag = !this->pauseFlag;
    return true;
}

bool Ti5RobotTeleoperate::StopTeleoperate(){
    if(!this->startFlag){
        std::cout<<"Teleoperation has ended! "<<std::endl;
    }else{
        this->startFlag = false;
    }

    return true;
}
