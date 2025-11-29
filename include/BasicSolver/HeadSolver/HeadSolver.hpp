#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <boost/optional.hpp>
#include <RobotType.hpp>
#include <source_path.h>
#include <JsonParser/JsonParser.hpp>
#include <MatrixUtils.hpp>

class HeadSolver
{
public:
    HeadSolver();
    ~HeadSolver();

    Eigen::Vector3d Solve(const Eigen::Matrix4d &mat);

    std::vector<int> GetJointsIndex(const RobotType::Type &type);

    std::vector<std::string> GetJointsName(const RobotType::Type &type);

    std::vector<RobotType::JointInfo> GetJointsInfo(const RobotType::Type &type);

private:

    void Init();

    Eigen::Matrix4d headPose;
    boost::optional<Eigen::VectorXd> rpy;

    double roll;
    double pitch;
    double yaw;

    std::string configPath;
    JsonParser jsonParser;
};

