#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <boost/optional.hpp>
#include <RobotType.hpp>
#include <source_path.h>
#include <JsonParser/JsonParser.hpp>
#include <MatrixUtils.hpp>

class WaistSolver
{
public:
    WaistSolver();
    WaistSolver(const RobotType::Type& type);
    ~WaistSolver();

    void Init(const RobotType::Type& type);

    Eigen::Vector3d Solve(const Eigen::Matrix4d &mat);

    std::vector<int> GetJointsIndex();

    std::vector<std::string> GetJointsName();

    std::vector<RobotType::JointInfo> GetJointsInfo();

private:
    Eigen::Matrix4d headPose;
    boost::optional<Eigen::VectorXd> rpy;

    double roll;
    double pitch;
    double yaw;

    std::string configPath;
    JsonParser jsonParser;
    json::object rootObj;
    json::object robotObj;

    std::string typeStr;

    std::vector<double> upperBound;
    std::vector<double> lowerBound;

    RobotType::Type type;

    std::vector<RobotType::JointInfo> jointsInfo;
};

