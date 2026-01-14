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

    struct HumanoidState{
        // q
        std::vector<double> qHead;
        std::vector<double> qLeftArm;
        std::vector<double> qRightArm;
        std::vector<double> qWaist;
        std::vector<double> qLeftLeg;
        std::vector<double> qRightLeg;
    };

    struct HumanoidCmd
    {
        bool enableHead = false;
        bool enableLeftArm = false;
        bool enableRightArm = false;
        bool enableWaist = false;
        bool enableLeftLeg = false;
        bool enableRightLeg = false;

        // q
        std::vector<double> qTargetHead;
        std::vector<double> qTargetLeftArm;
        std::vector<double> qTargetRightArm;
        std::vector<double> qTargetWaist;
        std::vector<double> qTargetLeftLeg;
        std::vector<double> qTargetRightLeg;
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
    virtual void Info() = 0;

    virtual void GetJointsStatus();

    /* ---------------- xxxxxxxxxxxxxxx ---------------- */

    virtual bool MoveJ(const RobotHardware::HumanoidCmd& cmd);

    virtual bool BackToInitPose(const RobotHardware::HumanoidCmd& cmd);

    virtual bool BackToZero(const RobotHardware::HumanoidCmd& cmd);

    virtual RobotHardware::HumanoidState GetState(const bool& verbose);

    virtual bool SendCmd(const RobotHardware::HumanoidCmd& cmd);

    /* ---------------- Static Method ---------------- */

    static boost::shared_ptr<RobotHardware> GetPtr(const RobotHardware::BasicConfig &config_);

    static boost::shared_ptr<RobotHardware> GetPtr(const std::string& filePath);


protected:

    RobotHardware::BasicConfig config;
};
