#include <G1Dof23Hardware/G1Dof23Hardware.hpp>

G1Dof23Hardware::G1Dof23Hardware(const RobotHardware::BasicConfig &config_){
    if(!this->CheckConfig(config_)){
        throw std::invalid_argument("[G1Dof23Hardware::G1Dof23Hardware] This config is wrong! ");
    }

    this->LoadConfig(config_);

    std::cout<<"The Description of the "<<RobotBase::GetStrFromType(this->config.robotType)<<std::endl;
    std::cout<<this->config.description<<std::endl;

//    this->LoadJointIndex();
    this->Init();

}

G1Dof23Hardware::~G1Dof23Hardware(){

}

/* ---------------- Motion Control ---------------- */

bool G1Dof23Hardware::MoveJ(const RobotHardware::RobotCmd& robotCmd)
{
    if(!this->initFlag){
        throw std::invalid_argument("[G1Dof23Hardware::MoveJ] Plz initialize first! ");
    }

    if(!this->CheckCmd(robotCmd)){
        throw std::invalid_argument("[G1Dof23Hardware::MoveJ] Invalid input robotCmd! ");
    }

    if(robotCmd.isHeadEnabled){
        throw std::invalid_argument("[G1Dof23Hardware::MoveJ] Sry, the UnitreeG1 doesn't have headDof! ");
    }

    std::unique_lock lock(this->mutex);

    if(robotCmd.isLeftArmEnabled){
        this->SetJointPosition(this->leftArmJointIndex,
                               robotCmd.qLeftArm);
    }

    if(robotCmd.isRightArmEnabled){
        this->SetJointPosition(this->rightArmJointIndex,
                               robotCmd.qRightArm);
    }

    if(robotCmd.isWaistEnabled){
        this->SetJointPosition(this->waistJointIndex,
                               robotCmd.qWaist);
    }

    if(robotCmd.isLeftLegEnabled){
        this->SetJointPosition(this->leftLegJointIndex,
                               robotCmd.qLeftLeg);
    }

    if(robotCmd.isRightLegEnabled){
        this->SetJointPosition(this->rightLegJointIndex,
                               robotCmd.qRightLeg);
    }

    return true;
}

bool G1Dof23Hardware::BackToInitPose(const RobotHardware::RobotCmd& robotCmd)
{
    return this->BackToZero(robotCmd);
}

bool G1Dof23Hardware::BackToZero(const RobotHardware::RobotCmd& robotCmd)
{
    RobotHardware::RobotCmd zeroCmd = robotCmd;

    if(robotCmd.isLeftArmEnabled)  zeroCmd.qLeftArm  = std::vector<double>(robotCmd.qLeftArm.size(), 0.0);
    if(robotCmd.isRightArmEnabled) zeroCmd.qRightArm = std::vector<double>(robotCmd.qRightArm.size(), 0.0);
    if(robotCmd.isWaistEnabled)    zeroCmd.qWaist    = std::vector<double>(robotCmd.qWaist.size(), 0.0);
    if(robotCmd.isLeftLegEnabled)  zeroCmd.qLeftLeg  = std::vector<double>(robotCmd.qLeftLeg.size(), 0.0);
    if(robotCmd.isRightLegEnabled) zeroCmd.qRightLeg = std::vector<double>(robotCmd.qRightLeg.size(), 0.0);

    return this->MoveJ(zeroCmd);
}

void G1Dof23Hardware::LoadConfig(const RobotHardware::BasicConfig &config_){

    this->config = config_;

}

bool G1Dof23Hardware::CheckConfig(const RobotHardware::BasicConfig &config_){
    std::string errorHead = "[G1Dof23Hardware::CheckConfig] ";

    // Check arm DOF
    if(config_.armDof != RobotBase::UnitreeG1::Dof23::armDof){
        std::ostringstream oss;
        oss << errorHead
            << "The armDof is " << config_.armDof
            << ", but it should be " << RobotBase::UnitreeG1::Dof23::armDof << "!";
        throw std::invalid_argument(oss.str());
    }

    // Check leg DOF
    if(config_.legDof != RobotBase::UnitreeG1::Dof23::legDof){
        std::ostringstream oss;
        oss << errorHead
            << "The legDof is " << config_.legDof
            << ", but it should be " << RobotBase::UnitreeG1::Dof23::legDof << "!";
        throw std::invalid_argument(oss.str());
    }

    // Check head DOF
    if(config_.headDof != RobotBase::UnitreeG1::Dof23::headDof){
        std::ostringstream oss;
        oss << errorHead
            << "The headDof is " << config_.headDof
            << ", but it should be " << RobotBase::UnitreeG1::Dof23::headDof << "!";
        throw std::invalid_argument(oss.str());
    }

    // Check waist DOF
    if(config_.waistDof != RobotBase::UnitreeG1::Dof23::waistDof){
        std::ostringstream oss;
        oss << errorHead
            << "The waistDof is " << config_.waistDof
            << ", but it should be " << RobotBase::UnitreeG1::Dof23::waistDof << "!";
        throw std::invalid_argument(oss.str());
    }

    // Check total DOF
    if(config_.totalDof != RobotBase::UnitreeG1::Dof23::totalDof){
        std::ostringstream oss;
        oss << errorHead
            << "The totalDof is " << config_.totalDof
            << ", but it should be " << RobotBase::UnitreeG1::Dof23::totalDof << "!";
        throw std::invalid_argument(oss.str());
    }

    return true;
}

bool G1Dof23Hardware::Init(){
    LOG_FUNCTION;
    if(this->initFlag){
        std::cout<<"[G1Dof23Hardware::Init] Have already initialzed! "<<std::endl;
        return true;
    }else{
        std::cout<<"[G1Dof23Hardware::Init] Start to initialize! "<<std::endl;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, this->config.networkInterface);

    // Publisher
    this->cmdPublisher.reset(
        new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(
            RobotBase::UnitreeG1::kTopicArmSDK));
    this->cmdPublisher->InitChannel();

    // Subscriber
    this->stateSubscriber.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
            RobotBase::UnitreeG1::kTopicState));
    this->stateSubscriber->InitChannel([&](const void *msg) {
          auto s = ( const unitree_hg::msg::dds_::LowState_* )msg;
          memcpy( &this->stateMsg, s, sizeof( unitree_hg::msg::dds_::LowState_ ) );
    }, 1);

    // TODO: InitMsg
    this->InitMsg();

    this->initFlag = true;
    return true;
}

//void G1Dof23Hardware::LoadJointIndex(){

//    this->leftArmJointIndex = {
//        RobotBase::UnitreeG1::JointIndex::kLeftShoulderPitch,
//        RobotBase::UnitreeG1::JointIndex::kLeftShoulderRoll,
//        RobotBase::UnitreeG1::JointIndex::kLeftShoulderYaw,
//        RobotBase::UnitreeG1::JointIndex::kLeftElbowPitch,
//        RobotBase::UnitreeG1::JointIndex::kLeftElbowRoll,
//    };

//    this->rightArmJointIndex = {
//        RobotBase::UnitreeG1::JointIndex::kRightShoulderPitch,
//        RobotBase::UnitreeG1::JointIndex::kRightShoulderRoll,
//        RobotBase::UnitreeG1::JointIndex::kRightShoulderYaw,
//        RobotBase::UnitreeG1::JointIndex::kRightElbowPitch,
//        RobotBase::UnitreeG1::JointIndex::kRightElbowRoll,
//    };

//    this->waistJointIndex = {
//        RobotBase::UnitreeG1::JointIndex::kWaistYaw,
//        RobotBase::UnitreeG1::JointIndex::kWaistRoll,
//        RobotBase::UnitreeG1::JointIndex::kWaistPitch,
//    };

//    this->leftLegJointIndex = {
//        RobotBase::UnitreeG1::JointIndex::kLeftHipPitch,
//        RobotBase::UnitreeG1::JointIndex::kLeftHipRoll,
//        RobotBase::UnitreeG1::JointIndex::kLeftHipYaw,
//        RobotBase::UnitreeG1::JointIndex::kLeftKnee,
//        RobotBase::UnitreeG1::JointIndex::kLeftAnkle,
//        RobotBase::UnitreeG1::JointIndex::kLeftAnkleRoll,
//    };

//    this->rightLegJointIndex = {
//        RobotBase::UnitreeG1::JointIndex::kRightHipPitch,
//        RobotBase::UnitreeG1::JointIndex::kRightHipRoll,
//        RobotBase::UnitreeG1::JointIndex::kRightHipYaw,
//        RobotBase::UnitreeG1::JointIndex::kRightKnee,
//        RobotBase::UnitreeG1::JointIndex::kRightAnkle,
//        RobotBase::UnitreeG1::JointIndex::kRightAnkleRoll,
//    };
//}

bool G1Dof23Hardware::CheckCmd(const RobotCmd &robotCmd)
{
    // Left Arm
    if (robotCmd.isLeftArmEnabled) {
        if (robotCmd.qLeftArm.size() != RobotBase::UnitreeG1::Dof23::armDof) {
            std::string error =
                "The size of qLeftArm is not equal to RobotBase::UnitreeG1::Dof23::armDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Right Arm
    if (robotCmd.isRightArmEnabled) {
        if (robotCmd.qRightArm.size() != RobotBase::UnitreeG1::Dof23::armDof) {
            std::string error =
                "The size of qRightArm is not equal to RobotBase::UnitreeG1::Dof23::armDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Head
    if (robotCmd.isHeadEnabled) {
        if (robotCmd.qHead.size() != RobotBase::UnitreeG1::Dof23::headDof) {
            std::string error =
                "The size of qHead is not equal to RobotBase::UnitreeG1::Dof23::headDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Waist
    if (robotCmd.isWaistEnabled) {
        if (robotCmd.qWaist.size() != RobotBase::UnitreeG1::Dof23::waistDof) {
            std::string error =
                "The size of qWaist is not equal to RobotBase::UnitreeG1::Dof23::waistDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Left Leg
    if (robotCmd.isLeftLegEnabled) {
        if (robotCmd.qLeftLeg.size() != RobotBase::UnitreeG1::Dof23::legDof) {
            std::string error =
                "The size of qLeftLeg is not equal to RobotBase::UnitreeG1::Dof23::legDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Right Leg
    if (robotCmd.isRightLegEnabled) {
        if (robotCmd.qRightLeg.size() != RobotBase::UnitreeG1::Dof23::legDof) {
            std::string error =
                "The size of qRightLeg is not equal to RobotBase::UnitreeG1::Dof23::legDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }
    return true;
}

void G1Dof23Hardware::SetJointPosition(const std::vector<size_t>& jointIndex,
                                       const std::vector<double>& q)
{
    if(jointIndex.size() != q.size())
        throw std::invalid_argument("[G1Dof23Hardware::SetJointPosition] jointIndex size != q size");

//    std::unique_lock lock(this->mutex);
    for(size_t i=0; i<jointIndex.size(); ++i){
        size_t idx = jointIndex[i];
        if(idx >= this->cmdMsg.motor_cmd().size())
            throw std::out_of_range("[G1Dof23Hardware::SetJointPosition] jointIndex out of range");

        auto& motor = this->cmdMsg.motor_cmd().at(idx);
        motor.mode() = 1;
        motor.q() = q[i];
        motor.dq() = this->dq;
//        motor.kd() = this->kd;
//        motor.kp() = this->kp;
        motor.tau() = this->tauFeedforward;

    }
}

void G1Dof23Hardware::InitMsg()
{
    std::unique_lock lock(this->mutex);
    // We need to set the q of joint NotUsedJoint to 1
    this->cmdMsg.motor_cmd().at(RobotBase::UnitreeG1::JointIndex::kNotUsedJoint).q() = 1;

    // mode_pr for the control of the leg : Ankle and AnkleRoll
    this->cmdMsg.mode_pr() = 0;

    // mode_machine for the mode of the robot: \
    // UnitreeG1(23) -- 4
    // UnitreeG1(29) -- 5
    // UnitreeG1(27,Lock the waist) -- 6
    this->cmdMsg.mode_machine() = 4;

    // Set Mode to 1
    for(size_t i=0;i<this->leftArmJointIndex.size();i++){
        this->cmdMsg.motor_cmd().at(this->leftArmJointIndex[i]).mode() = 1;

        // For Wrist
        if(this->leftArmJointIndex[i] >= RobotBase::UnitreeG1::JointIndex::kLeftWristRoll &&
           this->leftArmJointIndex[i] << RobotBase::UnitreeG1::JointIndex::kLeftWristYaw){
            this->cmdMsg.motor_cmd().at(this->leftArmJointIndex[i]).kp() = this->kpWrist;
            this->cmdMsg.motor_cmd().at(this->leftArmJointIndex[i]).kd() = this->kdWrist;
        } else {
            this->cmdMsg.motor_cmd().at(this->leftArmJointIndex[i]).kp() = this->kpLow;
            this->cmdMsg.motor_cmd().at(this->leftArmJointIndex[i]).kd() = this->kdLow;
        }
    }

    for(size_t i=0;i<this->rightArmJointIndex.size();i++){
        this->cmdMsg.motor_cmd().at(this->rightArmJointIndex[i]).mode() = 1;

        // For Wrist
        if(this->rightArmJointIndex[i] >= RobotBase::UnitreeG1::JointIndex::kRightWristRoll &&
           this->rightArmJointIndex[i] << RobotBase::UnitreeG1::JointIndex::kRightWristYaw){
            this->cmdMsg.motor_cmd().at(this->rightArmJointIndex[i]).kp() = this->kpWrist;
            this->cmdMsg.motor_cmd().at(this->rightArmJointIndex[i]).kd() = this->kdWrist;
        } else {
            this->cmdMsg.motor_cmd().at(this->rightArmJointIndex[i]).kp() = this->kpLow;
            this->cmdMsg.motor_cmd().at(this->rightArmJointIndex[i]).kd() = this->kdLow;
        }
    }

    for(size_t i=0;i<this->waistJointIndex.size();i++){
        this->cmdMsg.motor_cmd().at(this->waistJointIndex[i]).mode() = 1;
        this->cmdMsg.motor_cmd().at(this->waistJointIndex[i]).kd() = this->kdHigh;
        this->cmdMsg.motor_cmd().at(this->waistJointIndex[i]).kp() = this->kpHigh;
    }

    for(size_t i=0;i<this->leftLegJointIndex.size();i++){
        this->cmdMsg.motor_cmd().at(this->leftLegJointIndex[i]).mode() = 1;

        if(this->leftLegJointIndex[i] == RobotBase::UnitreeG1::JointIndex::kLeftAnklePitch){
            this->cmdMsg.motor_cmd().at(this->leftLegJointIndex[i]).kp() = this->kpLow;
            this->cmdMsg.motor_cmd().at(this->leftLegJointIndex[i]).kd() = this->kdLow;
        } else {
            this->cmdMsg.motor_cmd().at(this->leftLegJointIndex[i]).kp() = this->kpHigh;
            this->cmdMsg.motor_cmd().at(this->leftLegJointIndex[i]).kd() = this->kdHigh;
        }
    }

    for(size_t i=0;i<this->rightLegJointIndex.size();i++){
        this->cmdMsg.motor_cmd().at(this->rightLegJointIndex[i]).mode() = 1;

        if(this->rightLegJointIndex[i] == RobotBase::UnitreeG1::JointIndex::kRightAnklePitch){
            this->cmdMsg.motor_cmd().at(this->rightLegJointIndex[i]).kp() = this->kpLow;
            this->cmdMsg.motor_cmd().at(this->rightLegJointIndex[i]).kd() = this->kdLow;
        } else {
            this->cmdMsg.motor_cmd().at(this->rightLegJointIndex[i]).kp() = this->kpHigh;
            this->cmdMsg.motor_cmd().at(this->rightLegJointIndex[i]).kd() = this->kdHigh;
        }
    }
}

void G1Dof23Hardware::MainLoop()
{

    while(true){
        auto start = std::chrono::high_resolution_clock::now();

        {
            std::unique_lock lock(this->mutex);

            // Send Msg
            this->cmdMsg.crc() = RobotBase::UnitreeG1::Crc32Core((uint32_t *)&this->cmdMsg,
                                                                 (sizeof(this->cmdMsg) >> 2) - 1);
            this->cmdPublisher->Write(this->cmdMsg);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << " Main Loop 耗时: " << duration.count() << " ms" << std::endl;

        int framePeriod = this->dt * 1000;
        int sleepTime = framePeriod - duration.count();

        if(sleepTime > 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
        }
    }
}


