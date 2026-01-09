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

class G1Dof23Hardware
        : public RobotHardware
{
public:
    G1Dof23Hardware(const RobotHardware::BasicConfig &config_);
    ~G1Dof23Hardware();

    bool MoveJ(const RobotHardware::RobotCmd& robotCmd) override;

    bool BackToInitPose(const RobotHardware::RobotCmd& robotCmd) override;

    bool BackToZero(const RobotHardware::RobotCmd& robotCmd) override;

    bool Init() override;

private:

    void LoadConfig(const RobotHardware::BasicConfig &config_);

    bool CheckConfig(const RobotHardware::BasicConfig &config_);

//    void LoadJointIndex();

    bool CheckCmd(const RobotHardware::RobotCmd& robotCmd);

    void SetJointPosition(const std::vector<size_t>& jointIndex,
                          const std::vector<double>& q);

    void InitMsg();

    void MainLoop();

    // Unitree Variable
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>
        cmdPublisher;
    unitree_hg::msg::dds_::LowCmd_ cmdMsg;

    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
        stateSubscriber;
    unitree_hg::msg::dds_::LowState_ stateMsg;

    std::shared_mutex mutex;

    bool initFlag = false;

    float kpHigh = 300.f;
    float kpLow = 80.f;

    float kdHigh = 3.f;
    float kdLow = 3.f;

    float kpWrist = 40.f;
    float kdWrist = 1.5f;

    float dq = 0.f;
    float tauFeedforward = 0.f;

    float armVelocityLimit = 20.f;
    float dt = 1.0 / 250.0;

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
        RobotBase::UnitreeG1::JointIndex::kWaistRoll,
        RobotBase::UnitreeG1::JointIndex::kWaistPitch,
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
