#pragma once

#include <HandController/HandController.hpp>
#include "ROHRegisters.hpp"

class ROHandController
        :public HandController
{
public:
    ROHandController(const HandController::BasicConfig& config_);
    ~ROHandController();

    std::vector<double> GetJointsAngle() override;

    Eigen::VectorXd GetJointsAngleEigen() override;

    inline std::vector<double> GetJointsBoundsUpper() override{
        return this->fingersUpperBound;
    };

    inline std::vector<double> GetJointsBoundsLower() override{
        return this->fingersLowerBound;
    };

    bool SetJointsAngle(const Eigen::VectorXd& targetValue) override;

    bool BackToInitPose() override;

private:
    int numFingers;
    void Init();

    modbus_t* ctx;

    std::string device;
    int baudrate;
    char parity;
    int dataBits;
    int stopBits;
    int slaveID;

    Eigen::VectorXd initPose;
};
