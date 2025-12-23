#include <Ti5PhysicalRobot/Ti5PhysicalRobot.hpp>
Ti5PhysicalRobot::Ti5PhysicalRobot(const PhysicalRobot::BasicConfig &config_){
//    l_solver.gap=0.5;
//    r_solver.gap=0.5;
//    l_controller.max_vel=0.6;
//    r_controller.max_vel=0.6;

}

Ti5PhysicalRobot::~Ti5PhysicalRobot(){

}

bool Ti5PhysicalRobot::Connect(){
    // 如果是在成功连接之后调用的话
    // 就直接通过Flag来判断
    // 避免再次重复连接的操作
    if(this->connectFlag){
        std::cout<< GREEN
                 << typeid(*this).name()
                 << " is already connected ! "
                 << RESET << std::endl;
    }

    // Start to connect
    std::vector<string> productSerialNumbers = query_can();
    if (productSerialNumbers.empty())
    {
        std::cout << RED
                  << "Can't find any USB device, plz retry！"
                  << RESET << std::endl;
        return false;
    }
    else
    {
        std::cout << RED
                  << " The CAN device is Founded ! "
                  << RESET << std::endl;
        std::cout << CYAN
                  << "CAN device serial number："
                  << RESET;
        for (const string &serialNumber : productSerialNumbers)
        {
            std::cout << CYAN
                      << serialNumber
                      << RESET << std::endl;
        }
    }
    std::string ip = ip_address();
    cout << MAGENTA << " IP = " << ip << RESET << endl;

    bool startFlag = Start();
//    bool startFlag = CanDriver::InitCan();
//    ti5_socket_server(0,0,SERVER_PORT);

    if(startFlag){
        std::cout << GREEN
                    << " Successfully initialize the CAN ! "
                    << RESET << std::endl;

          this->connectFlag = true ;
    }else{
        std::cout << RED
                  << " Can't initialize the CAN ! "
                  << RESET << std::endl;
    }
    return this->connectFlag;
}

bool Ti5PhysicalRobot::Disconnect(){
    bool exitFlag = Exit();
    if(exitFlag){
          std::cout << RED
                    << " Successfully exited CAN ! "
                    << RESET << std:: endl;
          this->connectFlag = false;
    }
//    CanDriver::CloseCan();
    return this->connectFlag;
}

bool Ti5PhysicalRobot::isConnect(){
    if(this->connectFlag){
        std::cout<< RED << typeid(*this).name() << " is connected ! " << RESET << std::endl;
    } else {
        std::cout<< RED << typeid(*this).name() << " is not connected ! " << RESET << std::endl;
    }
    return this->connectFlag;
}

bool Ti5PhysicalRobot::EmergencyStop(){
    brake(LEFT_ARM,0,0);
    brake(RIGHT_ARM,1,0);
    std:cout << RED
             << " Emergency Stop !!! "
             << RESET << std::endl;
    return true;
}

bool Ti5PhysicalRobot::BackToZero(){
    if(!this->isConnect()){ return false; }

    return true;
}

bool Ti5PhysicalRobot::BackToInitPose(const PhysicalRobot::Ti5RobotConfig& config_){
    if(!this->isConnect()){ return false; }

    // for left arm
    if(config_.useLeftArm){
        this->SendRecvJoints(std::vector<double>{-0.72, -1.0, 0.57, -1.0, 0.83, 0, 0},
                             this->dofArm,
                             this->leftArmCanDevice,
                             this->leftArmCanID,
                             "Left Arm"
                             );
    }

    // for right arm
    if(config_.useRightArm){
        this->SendRecvJoints(std::vector<double>{0.72, 1.0, -0.57, 1.0, -0.83, 0, 0},
                             this->dofArm,
                             this->rightArmCanDevice,
                             this->rightArmCanID,
                             "Right Arm"
                             );
    }

    return true;

}

bool Ti5PhysicalRobot::BackToZero(const PhysicalRobot::Ti5RobotConfig &config_){
    if(!this->isConnect()){ return false; }

    // for left arm
    if(config_.useLeftArm){
        this->SendRecvJoints(std::vector<double>{ 0 , 0 , 0 , 0 , 0 , 0 , 0 },
                             this->dofArm,
                             this->leftArmCanDevice,
                             this->leftArmCanID,
                             "Left Arm"
                             );
    }

    // for right arm
    if(config_.useRightArm){
        this->SendRecvJoints(std::vector<double>{ 0 , 0 , 0 , 0 , 0 , 0 , 0 },
                             this->dofArm,
                             this->rightArmCanDevice,
                             this->rightArmCanID,
                             "Right Arm"
                             );
    }

//    // for head
//    if(config_.useHead){
//        this->SendRecvJoints(std::vector<double>{ 0 , 0 , 0 , 0 , 0 , 0 , 0 },
//                             this->dofHead,
//                             this->headCanDevice,
//                             this->headCanID,
//                             "Head"
//                             );
//    }

//    // for waist
//    if(config_.useWaist){
//        this->SendRecvJoints(std::vector<double>{ 0 , 0 , 0 , 0 , 0 , 0 , 0 },
//                             this->dofWaist,
//                             this->waistCanDevice,
//                             this->waistCanID,
//                             "Waist"
//                             );
//    }

    // for hand
    // TODO

    return true;
}

std::vector<double> Ti5PhysicalRobot::GetJointsAngle(){
//    LOG_FUNCTION;
    if(!this->isConnect()){ return {}; }

    std::vector<double> jointsAngle;
    double position;

    // base
    jointsAngle.push_back(0);

    // waist
    jointsAngle.insert(jointsAngle.end(),3,0);

    // left arm
    for(size_t i=0;i<this->dofArm;i++){
        CanDriver::RecvPosition(this->leftArmCanDevice,this->leftArmCanID[i],&position);
        jointsAngle.push_back(position);
    }

    // neck
    jointsAngle.insert(jointsAngle.end(),3,0);

    // right arm
    for(size_t i=0;i<this->dofArm;i++){
        CanDriver::RecvPosition(this->rightArmCanDevice,this->rightArmCanID[i],&position);
        jointsAngle.push_back(position);
    }

    return jointsAngle;
}

Eigen::VectorXd Ti5PhysicalRobot::GetJointsAngleEigen(){
//    LOG_FUNCTION;
    if(!this->isConnect()){ return {}; }

    std::vector<double> jointsAngle = this->GetJointsAngle();
    Eigen::VectorXd jointsAngleEigen =
            Eigen::VectorXd::Map(jointsAngle.data(),jointsAngle.size());

    return jointsAngleEigen;
}

void Ti5PhysicalRobot::Info(){
    LOG_FUNCTION;
    ArmSide armSide;

    std::cout<< RED
             << " ------------------------------------------ "
             << RESET << std::endl;

    // x86_64架构下的HumanoidDualArmSolver没有这两个变量的定义
    // aarch64的有，所以会报错
    // 真是个草台班子啊
//    std::cout<< " Left Arm ID: " << Left_arm_id<<std::endl;
//    std::cout<< " Right Arm ID: " << Right_arm_id<<std::endl;

    // for head
    {
        std::cout<< GREEN
                 << " Head's canDevice and canIndex : "
                 << RESET << std::endl;
        for(size_t i = 0;i < this->dofHead ;i++){
            std::cout << GREEN
                      << " canDevice: " << size_t(0)
                      << " canIndex " << this->headCanID[i]
                      << RESET << std::endl;
        }
    }

    std::cout<< RED
             << " ------------------------------------------ "
             << RESET << std::endl;

    // for Left Arm
    {
        std::cout<< YELLOW
                 << " Left arm's canDevice and canIndex : "
                 << RESET << std::endl;
        for(size_t i = 0;i < this->dofArm ;i++){
            std::cout << YELLOW
                      << " canDevice: " << size_t(0)
                      << " canIndex " << this->leftArmCanID[i]
                      << RESET << std::endl;
        }
    }

    std::cout<< RED
             << " ------------------------------------------ "
             << RESET << std::endl;

    // for Right Arm
    {
        std::cout<< BLUE
                 << " Right arm's canDevice and canIndex : "
                 << RESET << std::endl;
        for(size_t i = 0;i < this->dofArm ;i++){
            std::cout << BLUE
                      << " canDevice: " << size_t(0)
                      << " canIndex " << this->rightArmCanID[i]
                      << RESET << std::endl;
        }
    }

    std::cout<< RED
             << " ------------------------------------------ "
             << RESET << std::endl;

}

void Ti5PhysicalRobot::GetJointsStatus(){
    if(!this->isConnect()){ return; }
    int32_t dataList[this->dofArm];

    get_mechanicalarm_status(ArmSide::LEFT_ARM,0,0,dataList);

    for(size_t i = 0;i < this->dofArm;i++){
        std::cout<<" Left Arm Joint " <<i <<" Status: "<<dataList[i]<<std::endl;
    }

    get_mechanicalarm_status(ArmSide::RIGHT_ARM,1,0,dataList);

    for(size_t i = 0;i < this->dofArm;i++){
        std::cout<<" Right Arm Joint " <<i <<" Status: "<<dataList[i]<<std::endl;
    }
}

bool Ti5PhysicalRobot::MoveJ(const std::vector<double> &jointsAngle_){
//    std::cout << RED
//              << " Sorry,please use MoveJ(const PhysicalRobot::CrpRobotConfig& config_) to control the CrpRobot ! "
//              << RESET
//              << std::endl;
    if(!this->isConnect()){ return false; }
    float jointsAngle[this->dofArm];

    for(size_t i=0;i<this->dofArm;i++){
        jointsAngle[i]=jointsAngle_[i];
    }

    joint_to_move(ArmSide::LEFT_ARM,jointsAngle,0,0);

    return true;
}

bool Ti5PhysicalRobot::MoveL(){

    return true;
}

// Angle Unit: rad
bool Ti5PhysicalRobot::MoveJ(const PhysicalRobot::Ti5RobotConfig &config_){
    if(!this->isConnect()){ return false; }

    // for left arm
    if(config_.useLeftArm){
        this->SendRecvJoints(config_.leftArmJointsValue,
                             this->dofArm,
                             this->leftArmCanDevice,
                             this->leftArmCanID,
                             "Left Arm"
                             );
    }

    // for right arm
    if(config_.useRightArm){
        this->SendRecvJoints(config_.rightArmJointsValue,
                             this->dofArm,
                             this->rightArmCanDevice,
                             this->rightArmCanID,
                             "Right Arm"
                             );
    }

    // for head
    if(config_.useHead){
        this->SendRecvJoints(config_.headJointsValue,
                             this->dofHead,
                             this->headCanDevice,
                             this->headCanID,
                             "Head"
                             );
    }

    // for waist
    if(config_.useWaist){
        this->SendRecvJoints(config_.waistJointsValue,
                             this->dofWaist,
                             this->waistCanDevice,
                             this->waistCanID,
                             "Waist"
                             );
    }

    // for hand
    // TODO

    return true;
}

bool Ti5PhysicalRobot::Init(){
    LOG_FUNCTION;
    if(this->isConnect()){
        this->Initialize(true);
        return true;
    }else{
        std::cout<<"Init failed! "<<std::endl;
        return false;
    }
}

bool Ti5PhysicalRobot::Initialize(bool verbose){
    double maxVelocity = 800;
    double maxAcceleration = 5;
    double currentVelocity;
    double currentAcceleration;

    std::cout<< " ----- Config and get max velocity and acceleration ----- " <<std::endl;
    for(size_t i=0;i<this->dofArm;i++){
        // left arm
        CanDriver::ConfigMaxVelocity(this->leftArmCanDevice, this->leftArmCanID[i], maxVelocity);
        CanDriver::RecvMaxVelocity(this->leftArmCanDevice, this->leftArmCanID[i], &currentVelocity);

        CanDriver::ConfigMaxAcceleration(this->leftArmCanDevice, this->leftArmCanID[i], maxAcceleration);
        CanDriver::RecvMaxAcceleration(this->leftArmCanDevice, this->leftArmCanID[i], &currentAcceleration);

        if(verbose){
            std::cout<<"Left Arm Joint "<<i<<
                       " Max Velocity: "<<
                       std::fixed << std::setprecision(3)<<currentVelocity <<std::endl;
            std::cout<<"Left Arm Joint "<<i<<
                       " Max Acceleration: "<<
                       std::fixed << std::setprecision(3)<<currentAcceleration<<std::endl;
        }

        // right arm
        CanDriver::ConfigMaxVelocity(this->rightArmCanDevice, this->rightArmCanID[i], maxVelocity);
        CanDriver::RecvMaxVelocity(this->rightArmCanDevice, this->rightArmCanID[i], &currentVelocity);

        CanDriver::ConfigMaxAcceleration(this->rightArmCanDevice, this->rightArmCanID[i], maxAcceleration);
        CanDriver::RecvMaxAcceleration(this->rightArmCanDevice, this->rightArmCanID[i], &currentAcceleration);

        if(verbose){
            std::cout<<"Right Arm Joint "<<i<<
                       " Max Velocity: "<<
                       std::fixed << std::setprecision(3)<<currentVelocity <<std::endl;
            std::cout<<"Right Arm Joint "<<i<<
                       " Max Acceleration: "<<
                       std::fixed << std::setprecision(3)<<currentAcceleration<<std::endl;
            std::cout<< " --------------------------------------------- " <<std::endl;
        }
    }
    return true;
}

bool Ti5PhysicalRobot::SendRecvJoints(const std::vector<double>& jointsValue,
                    size_t dof,
                    size_t canDevice,
                    const std::vector<size_t>& canID,
                    const std::string& partName) {
    if (jointsValue.empty()) {
        throw std::invalid_argument("[" + partName + "] The joints vector is empty!");
        return false;
    } else if (jointsValue.size() != dof) {
        throw std::length_error("[" + partName + "] The joints vector size does not match!");
        return false;
    }

    for (size_t i = 0; i < dof; i++) {
        double value = jointsValue[i];
        auto result = CanDriver::SendRecvPosition(canDevice, canID[i], &value);
        if (result.has_value()) {
            std::cout << "[" << partName << "] "
                      << "Can ID: " << std::fixed << std::setprecision(4) << std::get<0>(result.value())
                      << " Value: " << std::fixed << std::setprecision(4) << std::get<1>(result.value())
                      << std::endl;
        } else {
            std::cerr << "[" << partName << "] "
                      << "Failed to send position for Can ID: " << canID[i]
                      << std::endl;
        }
    }
    return true;
}
