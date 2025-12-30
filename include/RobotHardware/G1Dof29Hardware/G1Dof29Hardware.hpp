#pragma once
#include <RobotHardware/RobotHardware.hpp>

class G1Dof29Hardware
        : public RobotHardware
{
public:
    G1Dof29Hardware(const RobotHardware::BasicConfig &config);
    ~G1Dof29Hardware();

    bool Connect() override;

    bool Disconnect() override;

    bool isConnect() override;

    bool MoveJ(const RobotHardware::RobotCmd& config) override;

    bool BackToInitPose(const RobotHardware::RobotCmd& config) override;

    bool BackToZero(const RobotHardware::RobotCmd& config) override;

private:


};
