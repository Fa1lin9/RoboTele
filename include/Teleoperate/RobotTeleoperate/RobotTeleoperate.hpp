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
        RobotHardware::BasicConfig robotConfig;
//        Transform::BasicConfig transformConfig;
        std::string transformConfigPath;
        Ros2Bridge::BasicConfig bridgeConfig;

        bool isSim;
        bool isReal;

        bool isCheckSolution;

        bool isFilterSolution;
        std::vector<double> filterWeight;

        bool useHead;
        bool useWaist;
    };

    RobotTeleoperate();
    ~RobotTeleoperate();

    static boost::shared_ptr<RobotTeleoperate> GetPtr(const RobotTeleoperate::BasicConfig& config_);

    static boost::shared_ptr<RobotTeleoperate> GetPtr(const std::string& filePath);

    virtual bool Init() = 0;
    virtual bool StartTeleop(bool verbose) = 0;
    virtual bool StopTeleop() = 0;

protected:
    // Solver
    boost::shared_ptr<ArmSolver> armSolverPtr;

    boost::shared_ptr<RobotHardware> physicalRobotPtr;

    boost::shared_ptr<Transform> transformPtr;

    int FPS;

    bool isSim;
    bool isReal;
    bool isCheckSolution;

    // Filter
    bool isFilterSolution;
    std::vector<double> filterWeight;

    // For Head and Waist Control
    HeadSolver headSolver;
    WaistSolver waistSolver;
    bool useHead;
    bool useWaist;
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
};
