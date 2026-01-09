#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <FunctionLogger.hpp>
#include <iostream>

#include <RobotBase.hpp>

#include <JsonParser/JsonParser.hpp>

class RobotHardware
{
public:
    struct BasicConfig
    {
        std::string IP;
        std::string networkInterface;

        RobotBase::RobotType robotType;

        std::string description;

        // For Dof
        size_t headDof;
        size_t armDof;
        size_t waistDof;
        size_t legDof;
        size_t totalDof;

    };

    struct RobotCmd
    {
        bool isLeftArmEnabled = false;
        bool isRightArmEnabled = false;
        bool isHeadEnabled = false;
        bool isWaistEnabled = false;
        bool isLeftLegEnabled = false;
        bool isRightLegEnabled = false;

        // q
        std::vector<double> qLeftArm;
        std::vector<double> qRightArm;
        std::vector<double> qHead;
        std::vector<double> qWaist;
        std::vector<double> qLeftLeg;
        std::vector<double> qRightLeg;
    };

    RobotHardware();
    ~RobotHardware();

    /* ---------------- Connection ---------------- */

    virtual bool Connect();

    virtual bool Disconnect();

    virtual bool isConnect();

    /* ---------------- Basic Action ---------------- */

    virtual bool Init();

    virtual bool EmergencyStop();

    virtual bool BackToZero();

    virtual bool MoveJ(const std::vector<double> &jointsAngle_);

    virtual bool MoveL();

    /* ---------------- Get Information ---------------- */

    virtual std::vector<double> GetJointsAngle();

    virtual Eigen::VectorXd GetJointsAngleEigen();

    // give some basic information of current robot
    virtual void Info();

    virtual void GetJointsStatus();

    /* ---------------- xxxxxxxxxxxxxxx ---------------- */

    virtual bool MoveJ(const RobotHardware::RobotCmd& config_) = 0;

    virtual bool BackToInitPose(const RobotHardware::RobotCmd& config_) = 0;

    virtual bool BackToZero(const RobotHardware::RobotCmd& config_) = 0;


    /* ---------------- Static Method ---------------- */

    static boost::shared_ptr<RobotHardware> GetPtr(const RobotHardware::BasicConfig &config_);

    static boost::shared_ptr<RobotHardware> GetPtr(const std::string& filePath);


protected:

    RobotHardware::BasicConfig config;
};
