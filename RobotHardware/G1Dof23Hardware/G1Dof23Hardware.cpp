#include <G1Dof23Hardware/G1Dof23Hardware.hpp>

G1Dof23Hardware::G1Dof23Hardware(const RobotHardware::BasicConfig &config_){
    if(!this->CheckConfig(config_)){
        throw std::invalid_argument("[G1Dof23Hardware::G1Dof23Hardware] This config is wrong! ");
    }

    this->LoadConfig(config_);

    std::cout<<"The Description of the "<<RobotBase::GetStrFromType(this->config.robotType)<<std::endl;
    std::cout<<this->config.description<<std::endl;

//    this->LoadJointIndex();
    this->Initialize();

    // Start MainLoop
    this->mainThread = std::thread(&G1Dof23Hardware::MainLoop, this);
}

G1Dof23Hardware::~G1Dof23Hardware(){
    if(this->mainThread.joinable()){
        this->mainThread.join();
    }
}

/* ---------------- Motion Control ---------------- */

bool G1Dof23Hardware::SendCmd(const RobotHardware::HumanoidCmd& cmd)
{
    if(!this->initFlag){
        throw std::invalid_argument("[G1Dof23Hardware::SendCmd] Plz initialize first! ");
    }

    if(!this->CheckCmd(cmd)){
        throw std::invalid_argument("[G1Dof23Hardware::SendCmd] Invalid input robotCmd! ");
    }

    if(cmd.isHeadEnabled){
        throw std::invalid_argument("[G1Dof23Hardware::SendCmd] Sry, the UnitreeG1 doesn't have headDof! ");
    }

    std::unique_lock lock(this->mutex);

    if(cmd.isLeftArmEnabled){
        this->SetJointPosition(this->leftArmJointIndex, cmd.qTargetLeftArm);
    }

    if(cmd.isRightArmEnabled){
        this->SetJointPosition(this->rightArmJointIndex, cmd.qTargetRightArm);
    }

    if(cmd.isWaistEnabled){
        this->SetJointPosition(this->waistJointIndex, cmd.qTargetWaist);
    }

    if(cmd.isLeftLegEnabled){
        this->SetJointPosition(this->leftLegJointIndex, cmd.qTargetLeftLeg);
    }

    if(cmd.isRightLegEnabled){
        this->SetJointPosition(this->rightLegJointIndex, cmd.qTargetRightLeg);
    }

//    this->cmdBuffer.SetData(cmd);

    return true;
}

bool G1Dof23Hardware::BackToInitPose(const RobotHardware::HumanoidCmd& robotCmd)
{
    return this->BackToZero(robotCmd);
}

bool G1Dof23Hardware::BackToZero(const RobotHardware::HumanoidCmd& robotCmd)
{
    RobotHardware::HumanoidCmd zeroCmd = robotCmd;

    if(robotCmd.isLeftArmEnabled)  zeroCmd.qTargetLeftArm  =
            std::vector<double>(this->armDof, 0.0);
    if(robotCmd.isRightArmEnabled) zeroCmd.qTargetRightArm =
            std::vector<double>(this->armDof, 0.0);
    if(robotCmd.isWaistEnabled)    zeroCmd.qTargetWaist    =
            std::vector<double>(this->waistDof, 0.0);
    if(robotCmd.isLeftLegEnabled)  zeroCmd.qTargetLeftLeg  =
            std::vector<double>(this->legDof, 0.0);
    if(robotCmd.isRightLegEnabled) zeroCmd.qTargetRightLeg =
            std::vector<double>(this->legDof, 0.0);

    return this->SendCmd(zeroCmd);
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
    this->Initialize();
    return true;
}

bool G1Dof23Hardware::Initialize()
{
    LOG_FUNCTION;
    if(this->initFlag){
        std::cout<<"[G1Dof23Hardware::Initialize] Have already initialzed! "<<std::endl;
        return true;
    }else{
        std::cout<<"[G1Dof23Hardware::Initialize] Start to initialize! "<<std::endl;
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
    this->stateSubscriber->InitChannel(
                std::bind(&G1Dof23Hardware::StateCallback, this, std::placeholders::_1), 1);

    // TODO: InitMsg
    this->InitMsg();

    this->initFlag = true;
    return true;
}

bool G1Dof23Hardware::CheckCmd(const HumanoidCmd &robotCmd)
{
    // Left Arm
    if (robotCmd.isLeftArmEnabled) {
        if (robotCmd.qTargetLeftArm.size() != RobotBase::UnitreeG1::Dof23::armDof) {
            std::string error =
                "The size of qLeftArm is not equal to RobotBase::UnitreeG1::Dof23::armDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Right Arm
    if (robotCmd.isRightArmEnabled) {
        if (robotCmd.qTargetRightArm.size() != RobotBase::UnitreeG1::Dof23::armDof) {
            std::string error =
                "The size of qRightArm is not equal to RobotBase::UnitreeG1::Dof23::armDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Head
    if (robotCmd.isHeadEnabled) {
        if (robotCmd.qTargetHead.size() != RobotBase::UnitreeG1::Dof23::headDof) {
            std::string error =
                "The size of qHead is not equal to RobotBase::UnitreeG1::Dof23::headDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Waist
    if (robotCmd.isWaistEnabled) {
        if (robotCmd.qTargetWaist.size() != RobotBase::UnitreeG1::Dof23::waistDof) {
            std::string error =
                "The size of qWaist is not equal to RobotBase::UnitreeG1::Dof23::waistDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Left Leg
    if (robotCmd.isLeftLegEnabled) {
        if (robotCmd.qTargetLeftLeg.size() != RobotBase::UnitreeG1::Dof23::legDof) {
            std::string error =
                "The size of qLeftLeg is not equal to RobotBase::UnitreeG1::Dof23::legDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Right Leg
    if (robotCmd.isRightLegEnabled) {
        if (robotCmd.qTargetRightLeg.size() != RobotBase::UnitreeG1::Dof23::legDof) {
            std::string error =
                "The size of qRightLeg is not equal to RobotBase::UnitreeG1::Dof23::legDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }
    return true;
}

void G1Dof23Hardware::SetJointPosition(const std::vector<size_t>& jointIndex,
                                       const std::vector<double>& qTarget)
{
    if(jointIndex.size() != qTarget.size())
        throw std::invalid_argument("[G1Dof23Hardware::SetJointPosition] jointIndex size != q size");

    for(size_t i=0; i<jointIndex.size(); ++i){
        size_t idx = jointIndex[i];
        if(idx >= this->cmdMsg.motor_cmd().size())
            throw std::out_of_range("[G1Dof23Hardware::SetJointPosition] jointIndex out of range");

        auto& motor = this->cmdMsg.motor_cmd().at(idx);
        motor.q() = qTarget[i];
        motor.dq() = this->dq;
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
            std::cout << " Time Sleep: " << sleepTime << " ms" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
        }
    }
}

void G1Dof23Hardware::StateCallback(const void *msg)
{
    unitree_hg::msg::dds_::LowState_ lowState =
        *(const unitree_hg::msg::dds_::LowState_ *)msg;
    RobotHardware::HumanoidState state;

    for(size_t i=0;i<this->leftArmJointIndex.size();i++){
        state.qLeftArm.push_back(lowState.motor_state().at(this->leftArmJointIndex[i]).q());
    }

    for(size_t i=0;i<this->rightArmJointIndex.size();i++){
        state.qRightArm.push_back(lowState.motor_state().at(this->rightArmJointIndex[i]).q());
    }

    for(size_t i=0;i<this->waistJointIndex.size();i++){
        state.qWaist.push_back(lowState.motor_state().at(this->waistJointIndex[i]).q());
    }

    for(size_t i=0;i<this->leftLegJointIndex.size();i++){
        state.qLeftLeg.push_back(lowState.motor_state().at(this->leftLegJointIndex[i]).q());
    }

    for(size_t i=0;i<this->rightLegJointIndex.size();i++){
        state.qRightLeg.push_back(lowState.motor_state().at(this->rightLegJointIndex[i]).q());
    }

    this->stateBuffer.SetData(state);

}

void G1Dof23Hardware::FinishWork()
{


}


