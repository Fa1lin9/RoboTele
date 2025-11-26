#pragma once

#include <iostream>
#include <modbus/modbus-rtu.h>
#include <modbus/modbus.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>

class HandController
{
public:
    enum Type{
        ROHand,

    };

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
        HandController::Type type;
        HandController::ModBusConfig modbusConfig;
    };

    HandController();
    ~HandController();

    virtual std::vector<double> GetJointsAngle();

    virtual Eigen::VectorXd GetJointsAngleEigen();

    virtual std::vector<double> GetJointsBoundsUpper();

    virtual std::vector<double> GetJointsBoundsLower();

    virtual bool SetJointsAngle(const Eigen::VectorXd& targetValue);

    virtual bool BackToInitPose();


    // static function

    static boost::shared_ptr<HandController> GetPtr(const HandController::BasicConfig& config_);

    static HandController::Type GetTypeFromStr(const std::string& str);

protected:
    static const std::unordered_map<std::string, HandController::Type> typeMap;

    std::vector<std::string> jointsName;
    std::vector<double> jointsBoundsUpper;
    std::vector<double> jointsBoundsLower;

};
