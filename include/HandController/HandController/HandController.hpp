#pragma once

#include <iostream>
#include <thread>
#include <chrono>

#include <modbus/modbus-rtu.h>
#include <modbus/modbus.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>

#include <HandBase.hpp>

class HandController
{
public:
    struct ModBusConfig
    {
        std::string device;
        int baudrate;
        std::string parity;
        int dataBits;
        int stopBits;
        int slaveID;
    };

    struct BasicConfig
    {
        HandBase::HandType type;
        HandController::ModBusConfig modbusConfig;
    };

    HandController();
    ~HandController();

    virtual std::vector<double> GetJointsAngle() = 0;

    virtual Eigen::VectorXd GetJointsAngleEigen() = 0;

    virtual std::vector<double> GetJointsBoundsUpper() = 0;

    virtual std::vector<double> GetJointsBoundsLower() = 0;

    virtual bool SetJointsAngle(const Eigen::VectorXd& targetValue) = 0;

    virtual bool BackToInitPose() = 0;


    // static function

    static boost::shared_ptr<HandController> GetPtr(const HandController::BasicConfig& config_);

protected:

    std::vector<std::string> jointsName;
    std::vector<double> jointsBoundsUpper;
    std::vector<double> jointsBoundsLower;

};
