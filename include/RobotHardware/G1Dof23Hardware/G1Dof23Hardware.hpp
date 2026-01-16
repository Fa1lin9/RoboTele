#pragma once
#include <RobotHardware/RobotHardware.hpp>
#include <math.h>
#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <error.h>
#include <mutex>
#include <shared_mutex>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

class G1Dof23Hardware
        : public RobotHardware
{
public:
    G1Dof23Hardware(const RobotHardware::BasicConfig &config_);
    ~G1Dof23Hardware();

    bool SendCmd(const RobotHardware::HumanoidCmd& robotCmd) override;

    RobotHardware::HumanoidState GetState(const bool& verbose) override;

    Eigen::VectorXd GetJointsAngleEigen() override;

    bool BackToInitPose(const RobotHardware::HumanoidCmd& robotCmd) override;

    bool BackToZero(const RobotHardware::HumanoidCmd& robotCmd) override;

    bool Init() override;

    void Info() override;

private:
    bool Initialize();

    void LoadConfig(const RobotHardware::BasicConfig &config_);

    bool CheckConfig(const RobotHardware::BasicConfig &config_);

    bool CheckCmd(const RobotHardware::HumanoidCmd& robotCmd);

    void SetJointPosition(const std::vector<double>& qTarget,
                          const std::vector<double>& qCurrent,
                          const std::vector<size_t>& jointIndex);

    void InitMsg();

    void MainLoop();

    void StateCallback(const void* msg);

    void FinishWork();

    void CommandWriter();

    std::vector<double> ClipJointAngles(const std::vector<double>& qTarget,
                                        const std::vector<double>& qCurrent,
                                        const float& velocityLimit);

    std::string queryServiceName(std::string form, std::string name);

    int queryMotionStatus();

    void PrintCmdMsg();

    bool IsBackToInitPose();

    // Unitree Variable
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>
        cmdPublisher;
    unitree_hg::msg::dds_::LowCmd_ cmdMsg;

    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
        stateSubscriber;
//    unitree_hg::msg::dds_::LowState_ stateMsg;

    unitree::common::ThreadPtr cmdWriterPtr;

    std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc;

    std::shared_mutex cmdMutex;
    std::shared_mutex stateMutex;

    bool initFlag = false;
    std::atomic<bool> runningMainLoop{true};

    float kpHigh = 300.f;
    float kpLow = 80.f;

    float kdHigh = 3.f;
    float kdLow = 3.f;

    float kpWrist = 40.f;
    float kdWrist = 1.5f;

    float dq = 0.f;
    float tauFeedforward = 0.f;

    float velocityLimit = 20.f;
    float dt = 1.0 / 250.0;

    // Dof
    size_t armDof = RobotBase::UnitreeG1::Dof23::armDof;
    size_t headDof = RobotBase::UnitreeG1::Dof23::headDof;
    size_t legDof = RobotBase::UnitreeG1::Dof23::legDof;
    size_t waistDof = RobotBase::UnitreeG1::Dof23::waistDof;
    size_t totalDof = RobotBase::UnitreeG1::Dof23::totalDof;

    // thread
    std::thread mainThread;

    // DataBuffer
    RobotBase::DataBuffer<RobotHardware::HumanoidState> stateBuffer;
    RobotBase::DataBuffer<RobotHardware::HumanoidCmd> cmdBuffer;

    // Struct
//    RobotHardware::HumanoidCmd cmd;
//    RobotHardware::HumanoidState state;

    // JointIndex
    std::vector<size_t> leftArmJointIndex = {
        RobotBase::UnitreeG1::JointIndex::kLeftShoulderPitch,
        RobotBase::UnitreeG1::JointIndex::kLeftShoulderRoll,
        RobotBase::UnitreeG1::JointIndex::kLeftShoulderYaw,
        RobotBase::UnitreeG1::JointIndex::kLeftElbow,
        RobotBase::UnitreeG1::JointIndex::kLeftWristRoll,
    };

    std::vector<size_t> rightArmJointIndex= {
        RobotBase::UnitreeG1::JointIndex::kRightShoulderPitch,
        RobotBase::UnitreeG1::JointIndex::kRightShoulderRoll,
        RobotBase::UnitreeG1::JointIndex::kRightShoulderYaw,
        RobotBase::UnitreeG1::JointIndex::kRightElbow,
        RobotBase::UnitreeG1::JointIndex::kRightWristRoll,
    };

    std::vector<size_t> waistJointIndex = {
        RobotBase::UnitreeG1::JointIndex::kWaistYaw,
//        RobotBase::UnitreeG1::JointIndex::kWaistRoll,
//        RobotBase::UnitreeG1::JointIndex::kWaistPitch,
    };

    std::vector<size_t> leftLegJointIndex = {
        RobotBase::UnitreeG1::JointIndex::kLeftHipPitch,
        RobotBase::UnitreeG1::JointIndex::kLeftHipRoll,
        RobotBase::UnitreeG1::JointIndex::kLeftHipYaw,
        RobotBase::UnitreeG1::JointIndex::kLeftKnee,
        RobotBase::UnitreeG1::JointIndex::kLeftAnklePitch,
        RobotBase::UnitreeG1::JointIndex::kLeftAnkleRoll,
    };

    std::vector<size_t> rightLegJointIndex = {
        RobotBase::UnitreeG1::JointIndex::kRightHipPitch,
        RobotBase::UnitreeG1::JointIndex::kRightHipRoll,
        RobotBase::UnitreeG1::JointIndex::kRightHipYaw,
        RobotBase::UnitreeG1::JointIndex::kRightKnee,
        RobotBase::UnitreeG1::JointIndex::kRightAnklePitch,
        RobotBase::UnitreeG1::JointIndex::kRightAnkleRoll,
    };
//    std::vector<size_t> headJointIndex;
};
