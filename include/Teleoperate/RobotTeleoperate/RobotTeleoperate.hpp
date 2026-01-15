#pragma once

#include <source_path.h>

#include <ArmSolver/ArmSolver.hpp>
#include <RobotHardware/RobotHardware.hpp>
#include <Transform/Transform.hpp>
#include <CsvWriter/CsvWriter.hpp>
#include <WeightedMovingFilter/WeightedMovingFilter.hpp>
#include <JsonParser/JsonParser.hpp>
#include <HeadSolver/HeadSolver.hpp>
#include <WaistSolver/WaistSolver.hpp>
#include <Ros2Bridge/Ros2Bridge.h>
#include <DataCollector/DataCollector.hpp>

// For Boost
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

// For Base
#include <RobotBase.hpp>
#include <XRBase.hpp>
#include <HandBase.hpp>

// For Hand
#include <HandSolver/HandSolver.hpp>
#include <HandController/HandController.hpp>
#include <HandGestureDetector/HandGestureDetector.hpp>

class RobotTeleoperate
{
public:
    struct BasicConfig{
        RobotBase::RobotType robotType;
        XRBase::XRType xrType;
        std::string address;
        int FPS;

//        ArmSolver::BasicConfig solverConfig;
        std::string solverConfigPath;
//        RobotHardware::BasicConfig hardwareConfig;
        std::string hardwareConfigPath;
//        Transform::BasicConfig transformConfig;
        std::string transformConfigPath;
        Ros2Bridge::BasicConfig bridgeConfig;

        bool isSim;
        bool isReal;

        bool isCheckSolution;

        bool isFilterSolution;
        std::vector<double> filterWeight;

        bool enableHead;
//        bool enableLeftArm;
//        bool enableRightArm;
        bool enableWaist;
//        bool enableLeftLeg;
//        bool enableRightLeg;
    };

    RobotTeleoperate();
    ~RobotTeleoperate();

    static boost::shared_ptr<RobotTeleoperate> GetPtr(const RobotTeleoperate::BasicConfig& config_);

    static boost::shared_ptr<RobotTeleoperate> GetPtr(const std::string& filePath);

    virtual bool Init();
    virtual bool StartTeleop(bool verbose);
    virtual bool StopTeleop();

    virtual void Info() = 0;

protected:
    // Solver
    boost::shared_ptr<ArmSolver> armSolverPtr;

    boost::shared_ptr<RobotHardware> hardwarePtr;

    boost::shared_ptr<Transform> transformPtr;

    // For Head and Waist Control
    HeadSolver headSolver;
    WaistSolver waistSolver;

    Eigen::Vector3d headRPY;
    Eigen::Vector3d waistRPY;
    std::vector<RobotBase::JointInfo> headJointsInfo;
    std::vector<RobotBase::JointInfo> waistJointsInfo;

    // Valuable
    std::vector<Eigen::Matrix4d> poseMatrix;

//    std::vector<Eigen::Vector3d> leftHandPositions;
//    std::vector<Eigen::Vector3d> rightHandPositions;

//    HandBase::HandGesture leftHandGesture;
//    HandBase::HandGesture rightHandGesture;
    HandBase::DualHandData dualHandData;

    HandGestureDetector handGestureDectector;

    Eigen::VectorXd qLast;

    RobotTeleoperate::BasicConfig config;
};
