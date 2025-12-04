#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <FunctionLogger.hpp>
#include <iostream>

#include <RobotBase.hpp>

class PhysicalRobot
{
public:
//    enum Type{
//        Ti5Robot
//    };

    struct BasicConfig
    {
        std::string IP;
        RobotBase::RobotType robotType;
    };

    struct Ti5RobotConfig
    {
        // some flag
        bool useLeftArm = false;
        bool useRightArm = false;
        bool useHead = false;
        bool useWaist = false;

        //
        std::vector<double> leftArmJointsValue;
        std::vector<double> rightArmJointsValue;
        std::vector<double> headJointsValue;
        std::vector<double> waistJointsValue;
    };

    PhysicalRobot();
    ~PhysicalRobot();

    /* ---------------- Connection ---------------- */

    virtual bool Connect() = 0;

    virtual bool Disconnect() = 0;

    virtual bool isConnect() = 0;

    /* ---------------- Basic Action ---------------- */

    virtual bool Init() = 0;

    virtual bool EmergencyStop() = 0;

    virtual bool BackToZero() = 0;

    virtual bool MoveJ(const std::vector<double> &jointsAngle_) = 0;

    virtual bool MoveL() = 0;

    /* ---------------- Get Information ---------------- */

    virtual std::vector<double> GetJointsAngle() = 0;

    virtual Eigen::VectorXd GetJointsAngleEigen() = 0;

    // give some basic information of current robot
    virtual void Info() = 0;

    virtual void GetJointsStatus() = 0;

    /* ---------------- Just for CrpRobot ---------------- */

    virtual bool MoveJ(const PhysicalRobot::Ti5RobotConfig& config_);

    virtual bool BackToInitPose(const PhysicalRobot::Ti5RobotConfig& config_);

    virtual bool BackToZero(const PhysicalRobot::Ti5RobotConfig& config_);


    /* ---------------- Static Method ---------------- */

    static boost::shared_ptr<PhysicalRobot> GetPtr(const PhysicalRobot::BasicConfig &config_);

private:

};
