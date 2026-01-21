#include <G1Dof23Hardware/G1Dof23Hardware.hpp>

G1Dof23Hardware::G1Dof23Hardware(const RobotHardware::BasicConfig &config_){
    if(!this->CheckConfig(config_)){
        throw std::invalid_argument("[G1Dof23Hardware::G1Dof23Hardware] This config is wrong! ");
    }

    this->LoadConfig(config_);

    std::cout<<"The Description of the "<<RobotBase::GetStrFromType(this->config.robotType)<<":"<<std::endl;
    std::cout<<this->config.description<<std::endl;

//    this->LoadJointIndex();
    this->Initialize();
}

G1Dof23Hardware::~G1Dof23Hardware(){
    this->runningMainLoop = false;
    if(this->mainThread.joinable()){
        std::cout << "[G1Dof23Hardware::~G1Dof23Hardware] Delete mainThread! " << std::endl;
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

    if(cmd.enableHead){
        throw std::invalid_argument("[G1Dof23Hardware::SendCmd] Sry, the UnitreeG1 doesn't have headDof! ");
    }

    if(cmd.verbose){
        std::cout << "[G1Dof23Hardware::SendCmd] Send cmd! " << std::endl;
    }

    this->cmdBuffer.SetData(cmd);
    std::unique_lock lock(this->cmdMutex);

    auto humanoidState = this->GetState(false);

    if(cmd.enableLeftArm){
//        std::cout << "[G1Dof23Hardware::SendCmd] Controlling Left Arm" << std::endl;
        this->SetJointPosition(cmd.qTargetLeftArm,
                               humanoidState.qLeftArm,
                               this->leftArmJointIndex);
    }

    if(cmd.enableRightArm){
//        std::cout << "[G1Dof23Hardware::SendCmd] Controlling Right Arm" << std::endl;
        this->SetJointPosition(cmd.qTargetRightArm,
                               humanoidState.qRightArm,
                               this->rightArmJointIndex);
    }

    if(cmd.enableWaist){
//        std::cout << "[G1Dof23Hardware::SendCmd] Controlling Waist" << std::endl;
        this->SetJointPosition(cmd.qTargetWaist,
                               humanoidState.qWaist,
                               this->waistJointIndex);
    }

    if(cmd.enableLeftLeg){
//        std::cout << "[G1Dof23Hardware::SendCmd] Controlling Left Leg" << std::endl;
        this->SetJointPosition(cmd.qTargetLeftLeg,
                               humanoidState.qLeftLeg,
                               this->leftLegJointIndex);
    }

    if(cmd.enableRightLeg){
//        std::cout << "[G1Dof23Hardware::SendCmd] Controlling Right Leg" << std::endl;
        this->SetJointPosition(cmd.qTargetRightLeg,
                               humanoidState.qRightLeg,
                               this->rightLegJointIndex);
    }

    return true;
}

RobotHardware::HumanoidState G1Dof23Hardware::GetState(const bool& verbose)
{
//    std::unique_lock lock(stateMutex);
    auto statePtr = this->stateBuffer.GetData();

    if(!statePtr){
        std::cerr << "[G1Dof23Hardware::GetState] stateBuffer returned null! " << std::endl;

        return RobotHardware::HumanoidState();
    } else {
        auto state = *statePtr;

        if(verbose){
            std::cout << "[G1Dof23Hardware::GetState] Get the state of the robot! " << std::endl;
            std::cout << " Current qLeftArm: " << std::endl;
            for(size_t i=0;i<this->armDof;i++){
                std::cout << state.qLeftArm[i] << std::endl;
            }
            std::cout << " Current qRightArm: " << std::endl;
            for(size_t i=0;i<this->armDof;i++){
                std::cout << state.qRightArm[i] << std::endl;
            }
        //    std::cout << " Current qWaist: " << std::endl;
        //    for(size_t i=0;i<this->waistDof;i++){
        //        std::cout << state.qWaist[i] << std::endl;
        //    }
        //    std::cout << " Current qLeftLeg: " << std::endl;
        //    for(size_t i=0;i<this->legDof;i++){
        //        std::cout << state.qLeftLeg[i] << std::endl;
        //    }
        //    std::cout << " Current qRightLeg: " << std::endl;
        //    for(size_t i=0;i<this->legDof;i++){
        //        std::cout << state.qRightLeg[i] << std::endl;
        //    }
        }

        return state;
    }
}

Eigen::VectorXd G1Dof23Hardware::GetJointsAngleEigen()
{
    auto statePtr = this->stateBuffer.GetData();

    if(!statePtr){
        std::cerr << "[G1Dof23Hardware::GetJointsAngleEigen] stateBuffer returned null! " << std::endl;

        return Eigen::VectorXd::Zero(this->totalDof);
    } else {
        auto state = *statePtr;

        Eigen::VectorXd res(this->totalDof);

        size_t totalSize =
                state.qLeftArm.size() + state.qRightArm.size() +
                state.qWaist.size() +
                state.qLeftLeg.size() + state.qRightLeg.size();
//        std::cout << "[G1Dof23Hardware::GetJointsAngleEigen] The totalSize is " << totalSize << "! " << std::endl;

        assert( totalSize == this->totalDof );

        size_t idx = 0;
        for(double val : state.qLeftLeg) res(idx++) = val;
        for(double val : state.qRightLeg) res(idx++) = val;
        for(double val : state.qWaist) res(idx++) = val;
        for(double val : state.qLeftArm) res(idx++) = val;
        for(double val : state.qRightArm) res(idx++) = val;

        assert(res.size() == this->totalDof);

//        std::cout << "[G1Dof23Hardware::GetJointsAngleEigen] q = " << res.transpose() << std::endl;

        return res;
    }

}

bool G1Dof23Hardware::BackToInitPose(const RobotHardware::HumanoidCmd& robotCmd)
{
    std::cout << "[G1Dof23Hardware::BackToInitPose] Back to initial pose! " << std::endl;
    return this->BackToZero(robotCmd);
}

bool G1Dof23Hardware::BackToZero(const RobotHardware::HumanoidCmd& robotCmd)
{
    RobotHardware::HumanoidCmd zeroCmd = robotCmd;

    if(robotCmd.enableLeftArm)  zeroCmd.qTargetLeftArm  =
            std::vector<double>(this->armDof, 0.0);
    if(robotCmd.enableRightArm) zeroCmd.qTargetRightArm =
            std::vector<double>(this->armDof, 0.0);
    if(robotCmd.enableWaist)    zeroCmd.qTargetWaist    =
            std::vector<double>(this->waistDof, 0.0);
    if(robotCmd.enableLeftLeg)  zeroCmd.qTargetLeftLeg  =
            std::vector<double>(this->legDof, 0.0);
    if(robotCmd.enableRightLeg) zeroCmd.qTargetRightLeg =
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

void G1Dof23Hardware::Info()
{
    LOG_FUNCTION;
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

    std::cout << "[G1Dof23Hardware::Initialize] networkInterface: " <<
                 this->config.networkInterface << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, this->config.networkInterface);

    this->msc.reset(new unitree::robot::b2::MotionSwitcherClient());
    this->msc->SetTimeout(5.0F);
    this->msc->Init();

    // Shut down motion control-related service
    while(queryMotionStatus())
    {
        std::cout << "[G1Dof23Hardware::Initialize] Try to deactivate the motion control-related service." << std::endl;
        int32_t ret = this->msc->ReleaseMode();
        if (ret == 0) {
            std::cout << "[G1Dof23Hardware::Initialize] ReleaseMode succeeded." << std::endl;
        } else {
            std::cout << "[G1Dof23Hardware::Initialize] ReleaseMode failed. Error code: " << ret << std::endl;
        }
        sleep(5);
    }

    // Publisher
    this->cmdPublisher.reset(
        new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(
            RobotBase::UnitreeG1::kTopicCmd));
    this->cmdPublisher->InitChannel();

    // Subscriber
    stateSubscriber.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
            RobotBase::UnitreeG1::kTopicState));
    stateSubscriber->InitChannel(
                std::bind(&G1Dof23Hardware::StateCallback, this, std::placeholders::_1), 1);

    // Must sleep some time to wait the subscriber have the state
    while(not this->stateBuffer.GetData()){
        std::cout<<"[G1Dof23Hardware::Initialize] Waiting to subscribe dds... "<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout<<"[G1Dof23Hardware::Initialize] Subscribe dds ok! "<<std::endl;

    // InitMsg
    this->InitMsg();

    // CommandWriter
//    this->cmdWriterPtr =
//        unitree::common::CreateRecurrentThreadEx(
//            "CommandWriter",
//            UT_CPU_ID_NONE,
//            static_cast<uint64_t>(this->dt * 1e6), // microsecond
//            &G1Dof23Hardware::CommandWriter,
//            this
//        );

    // Start MainLoop
    this->mainThread = std::thread(&G1Dof23Hardware::MainLoop, this);

    this->initFlag = true;
    return true;
}

bool G1Dof23Hardware::CheckCmd(const HumanoidCmd &robotCmd)
{
    // Left Arm
    if (robotCmd.enableLeftArm) {
        if (robotCmd.qTargetLeftArm.size() != RobotBase::UnitreeG1::Dof23::armDof) {
            std::string error =
                "The size of qLeftArm is not equal to RobotBase::UnitreeG1::Dof23::armDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Right Arm
    if (robotCmd.enableRightArm) {
        if (robotCmd.qTargetRightArm.size() != RobotBase::UnitreeG1::Dof23::armDof) {
            std::string error =
                "The size of qRightArm is not equal to RobotBase::UnitreeG1::Dof23::armDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Head
    if (robotCmd.enableHead) {
        if (robotCmd.qTargetHead.size() != RobotBase::UnitreeG1::Dof23::headDof) {
            std::string error =
                "The size of qHead is not equal to RobotBase::UnitreeG1::Dof23::headDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Waist
    if (robotCmd.enableWaist) {
        if (robotCmd.qTargetWaist.size() != RobotBase::UnitreeG1::Dof23::waistDof) {
            std::string error =
                "The size of qWaist is not equal to RobotBase::UnitreeG1::Dof23::waistDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Left Leg
    if (robotCmd.enableLeftLeg) {
        if (robotCmd.qTargetLeftLeg.size() != RobotBase::UnitreeG1::Dof23::legDof) {
            std::string error =
                "The size of qLeftLeg is not equal to RobotBase::UnitreeG1::Dof23::legDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }

    // Right Leg
    if (robotCmd.enableRightLeg) {
        if (robotCmd.qTargetRightLeg.size() != RobotBase::UnitreeG1::Dof23::legDof) {
            std::string error =
                "The size of qRightLeg is not equal to RobotBase::UnitreeG1::Dof23::legDof!";
            throw std::invalid_argument(error);
            return false;
        }
    }
    return true;
}

void G1Dof23Hardware::SetJointPosition(const std::vector<double>& qTarget,
                                       const std::vector<double>& qCurrent,
                                       const std::vector<size_t>& jointIndex)
{
    if(jointIndex.size() != qTarget.size())
        throw std::invalid_argument("[G1Dof23Hardware::SetJointPosition] jointIndex size != q size");

    // Get qCliped
    auto qCliped = this->ClipJointAngles(qTarget,
                                         qCurrent,
                                         this->velocityLimit);

    for(size_t i=0; i<jointIndex.size(); ++i){
        size_t idx = jointIndex[i];
        if(idx >= this->cmdMsg.motor_cmd().size())
            throw std::out_of_range("[G1Dof23Hardware::SetJointPosition] jointIndex out of range");

        auto& motor = this->cmdMsg.motor_cmd().at(idx);
//        motor.q() = qTarget[i];
        motor.q() = qCliped[i];
        motor.dq() = this->dq;
        motor.tau() = this->tauFeedforward;

    }
}

void G1Dof23Hardware::InitMsg()
{
    std::unique_lock lock(this->cmdMutex);
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
        if(this->leftArmJointIndex[i] >= RobotBase::UnitreeG1::JointIndex::kLeftWristRoll){
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
        if(this->rightArmJointIndex[i] >= RobotBase::UnitreeG1::JointIndex::kRightWristRoll){
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
    // Back to initial pose
    RobotHardware::HumanoidCmd cmd = {
        .enableHead = false,
        .enableLeftArm = true,
        .enableRightArm = true,
        .enableWaist = true,
        .enableLeftLeg = true,
        .enableRightLeg = true,
        .verbose = false,
    };

    auto initStart = std::chrono::steady_clock::now();
    bool initFlag = false;

    while (this->runningMainLoop) {
        auto start = std::chrono::high_resolution_clock::now();

        // Back to initial pose
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - initStart).count();

        if (elapsed >= 3.0 && !initFlag) {
            std::cout << "[G1Dof23Hardware::MainLoop] BackToInitPose finished." << std::endl;
            initFlag = true;
        }

        if (elapsed < 3.0) {
            this->BackToInitPose(cmd);
        }

        {
            std::unique_lock lock(this->cmdMutex);

//            this->PrintCmdMsg();

            // Send Msg
            this->cmdMsg.crc() = RobotBase::UnitreeG1::Crc32Core(
                (uint32_t *)&this->cmdMsg,
                (sizeof(this->cmdMsg) >> 2) - 1
            );
            this->cmdPublisher->Write(this->cmdMsg);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        int framePeriod = this->dt * 1000;
        int sleepTime = framePeriod - duration.count();

        if (sleepTime > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
        }
    }

}

void G1Dof23Hardware::StateCallback(const void *msg)
{
//    std::unique_lock lock(stateMutex);
    unitree_hg::msg::dds_::LowState_ lowState =
        *(const unitree_hg::msg::dds_::LowState_ *)msg;
    RobotHardware::HumanoidState state;

    if (lowState.crc() !=
        RobotBase::UnitreeG1::Crc32Core((uint32_t *)&lowState,
                  (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
      std::cout << "[G1Dof23Hardware::StateCallback] lowState CRC Error" << std::endl;
      return;
    }

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

//    std::cout << "------------------------------------------------" << std::endl;
//    std::cout << " Current qLeftArm: " << std::endl;
//    for(size_t i=0;i<this->armDof;i++){
//        std::cout << state.qLeftArm[i] << std::endl;
//    }
//    std::cout << " Current qRightArm: " << std::endl;
//    for(size_t i=0;i<this->armDof;i++){
//        std::cout << state.qRightArm[i] << std::endl;
//    }
//    std::cout << " Current qWaist: " << std::endl;
//    for(size_t i=0;i<this->waistDof;i++){
//        std::cout << state.qWaist[i] << std::endl;
//    }
//    std::cout << " Current qLeftLeg: " << std::endl;
//    for(size_t i=0;i<this->legDof;i++){
//        std::cout << state.qLeftLeg[i] << std::endl;
//    }
//    std::cout << " Current qRightLeg: " << std::endl;
//    for(size_t i=0;i<this->legDof;i++){
//        std::cout << state.qRightLeg[i] << std::endl;
//    }

    this->stateBuffer.SetData(state);
}

void G1Dof23Hardware::FinishWork()
{


}

void G1Dof23Hardware::CommandWriter()
{
//    std::unique_lock lock(this->cmdMutex);
    auto cmdPtr = (this->cmdBuffer.GetData());

    if (!cmdPtr) {
        std::cout << "[G1Dof23Hardware::CommandWriter] cmdBuffer is empty, skipping..." << std::endl;
        return;
    }

    const auto &cmd = *cmdPtr;

    auto humanoidState = this->GetState(false);

    if(cmd.enableLeftArm){
        std::cout << "[G1Dof23Hardware::SendCmd] Controlling Left Arm" << std::endl;
        this->SetJointPosition(cmd.qTargetLeftArm,
                               humanoidState.qLeftArm,
                               this->leftArmJointIndex);
    }

    if(cmd.enableRightArm){
        std::cout << "[G1Dof23Hardware::SendCmd] Controlling Right Arm" << std::endl;
        this->SetJointPosition(cmd.qTargetRightArm,
                               humanoidState.qRightArm,
                               this->rightArmJointIndex);
    }

    if(cmd.enableWaist){
        std::cout << "[G1Dof23Hardware::SendCmd] Controlling Waist" << std::endl;
        this->SetJointPosition(cmd.qTargetWaist,
                               humanoidState.qWaist,
                               this->waistJointIndex);
    }

    if(cmd.enableLeftLeg){
        std::cout << "[G1Dof23Hardware::SendCmd] Controlling Left Leg" << std::endl;
        this->SetJointPosition(cmd.qTargetLeftLeg,
                               humanoidState.qLeftLeg,
                               this->leftLegJointIndex);
    }

    if(cmd.enableRightLeg){
        std::cout << "[G1Dof23Hardware::SendCmd] Controlling Right Leg" << std::endl;
        this->SetJointPosition(cmd.qTargetRightLeg,
                               humanoidState.qRightLeg,
                               this->rightLegJointIndex);
    }

    // Send Msg
    this->cmdMsg.crc() = RobotBase::UnitreeG1::Crc32Core((uint32_t *)&this->cmdMsg,
                                                         (sizeof(this->cmdMsg) >> 2) - 1);
    this->cmdPublisher->Write(this->cmdMsg);
}

std::vector<double> G1Dof23Hardware::ClipJointAngles(const std::vector<double>& qTarget,
                                                     const std::vector<double>& qCurrent,
                                                     const float& velocityLimit)
{
    if(qTarget.size() != qCurrent.size()){
        throw std::invalid_argument("[G1Dof23Hardware::ClipJointAngles] Size match error! ");
    }

    std::vector<double> delta(qTarget.size());
    double max = 0.0;
    for(size_t i=0;i<qTarget.size();i++){
        delta[i] = qTarget[i] - qCurrent[i];
        max = std::max(max, std::abs(delta[i]));
    }

    auto motionScale = max / ( this->dt * velocityLimit );

//    std::cout << "[G1Dof23Hardware::ClipJointAngles] motionScale is " << motionScale << std::endl;

    std::vector<double> qCliped(qTarget.size());
    for(size_t i=0;i<delta.size();i++){
        qCliped[i] = qCurrent[i] + delta[i] / std::max(1.0, motionScale);
    }

    return qCliped;
}

std::string G1Dof23Hardware::queryServiceName(std::string form, std::string name)
{
    if(form == "0")
    {
        if(name == "normal" ) return "sport_mode";
        if(name == "ai" ) return "ai_sport";
        if(name == "advanced" ) return "advanced_sport";
    }
    else
    {
        if(name == "ai-w" ) return "wheeled_sport(go2W)";
        if(name == "normal-w" ) return "wheeled_sport(b2W)";
    }
    return "";
}

int G1Dof23Hardware::queryMotionStatus()
{
    std::string robotForm,motionName;
    int motionStatus;
    int32_t ret = msc->CheckMode(robotForm,motionName);
    if (ret == 0) {
        std::cout << "[G1Dof23Hardware::queryMotionStatus] CheckMode succeeded." << std::endl;
    } else {
        std::cout << "[G1Dof23Hardware::queryMotionStatus] CheckMode failed. Error code: " << ret << std::endl;
    }
    if(motionName.empty())
    {
        std::cout << "[G1Dof23Hardware::queryMotionStatus] The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = this->queryServiceName(robotForm,motionName);
        std::cout << "[G1Dof23Hardware::queryMotionStatus] Service: "<< serviceName<< " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

void G1Dof23Hardware::PrintCmdMsg()
{
    std::cout << "========== CmdMsg ==========" << std::endl;

    for (size_t i = 0; i < this->cmdMsg.motor_cmd().size(); ++i) {
        const auto &m = this->cmdMsg.motor_cmd().at(i);

        std::cout << "Motor[" << i << "]: "
                  << "q=" << m.q()
                  << ", dq=" << m.dq()
                  << ", tau=" << m.tau()
                  << ", mode=" << static_cast<int>(m.mode())
                  << std::endl;
    }

    std::cout << "============================" << std::endl;
}

bool G1Dof23Hardware::IsBackToInitPose()
{
    auto state = this->GetState(false);
    // TODO
    return false;
}




