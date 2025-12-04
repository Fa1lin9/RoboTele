#pragma once

#include <source_path.h>

#include <ArmSolver/ArmSolver.hpp>
#include <PhysicalRobot/PhysicalRobot.hpp>
#include <Transform/Transform.hpp>
#include <CsvWriter/CsvWriter.hpp>
#include <WeightedMovingFilter/WeightedMovingFilter.hpp>
#include <JsonParser/JsonParser.hpp>
#include <HeadSolver/HeadSolver.hpp>
#include <WaistSolver/WaistSolver.hpp>
#include <Ros2Bridge/Ros2Bridge.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <RobotBase.hpp>

#include <DataCollector/DataCollector.hpp>

class RobotTeleoperate
{
public:
//    enum Type{
//        Ti5Robot
//    };

    struct BasicConfig{
        RobotBase::RobotType robotType;
        std::string address;
        int FPS;

        ArmSolver::BasicConfig solverConfig;
        PhysicalRobot::BasicConfig robotConfig;
        Transform::BasicConfig transformConfig;
        Ros2Bridge::BasicConfig bridgeConfig;

        bool isSim;
        bool isReal;
    };

    RobotTeleoperate();
    ~RobotTeleoperate();

    static boost::shared_ptr<RobotTeleoperate> GetPtr(const RobotTeleoperate::BasicConfig& config_);

    static boost::shared_ptr<RobotTeleoperate> GetPtr(const std::string& filePath);

    virtual bool Init() = 0;
    virtual bool StartTeleoperate(bool verbose) = 0;
    virtual bool PauseTeleoperate() = 0;
    virtual bool StopTeleoperate() = 0;

protected:
    boost::shared_ptr<ArmSolver> ikSolverPtr;

    boost::shared_ptr<PhysicalRobot> physicalRobotPtr;

    boost::shared_ptr<Transform> transformPtr;

    HeadSolver headSolver;

    WaistSolver waistSolver;

    int FPS;

    bool isSim;
    bool isReal;

    // Valuable
    std::vector<Eigen::Matrix4d> poseMatrix;

    std::vector<Eigen::Vector3d> leftHandPositions;
    std::vector<Eigen::Vector3d> rightHandPositions;

    DataCollector::HandGesture leftHandGesture;
    DataCollector::HandGesture rightHandGesture;

};
