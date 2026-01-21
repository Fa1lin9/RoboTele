#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <boost/optional.hpp>
#include <RobotBase.hpp>
#include <source_path.h>
#include <JsonParser/JsonParser.hpp>
#include <MatrixUtils.hpp>

class WaistSolver
{
public:
    WaistSolver();
    WaistSolver(const std::string& path);
    ~WaistSolver();

    void Init(const std::string& path);

    Eigen::Vector3d Solve(const Eigen::Matrix4d &mat);

//    std::vector<int> GetJointsIndex();

//    std::vector<std::string> GetJointsName();

//    std::vector<int> GetDirection();

    std::vector<RobotBase::JointInfo> GetJointsInfo();

private:
    Eigen::Matrix4d headPose;
    boost::optional<Eigen::VectorXd> rpy;

    double roll;
    double pitch;
    double yaw;

    std::string configPath;
    JsonParser jsonParser;
    json::object robotObj;

//    std::string typeStr;
    std::vector<int> direction;
    std::vector<int> jointsIndex;
    std::vector<std::string> jointsName;

    std::vector<double> upperBound;
    std::vector<double> lowerBound;

    bool initFlag = false;

//    RobotBase::RobotType type;

    std::vector<RobotBase::JointInfo> jointsInfo;
};

