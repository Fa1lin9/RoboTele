#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <boost/make_shared.hpp>

class Transform
{
public:
    enum Type{
        VisionPro2Ti5Robot
    };

    struct BasicConfig
    {
        Eigen::Matrix4d                         T_Head2Waist;//头到机器人腰

        Eigen::Matrix4d                         T_XR2Robot;//OpenXR到机器人
        Eigen::Matrix4d                         T_Robot2LeftWrist;//机器人基坐标系到手腕
        Eigen::Matrix4d                         T_Robot2RightWrist;//机器人基坐标系到手腕

        Eigen::Vector3d                         offset;

        Transform::Type type;
    };

    struct MsgConfig{
        // 头和双臂手腕基于XR设备世界坐标系的位姿矩阵
        Eigen::Matrix4d                 head2xrWorldPose;
        Eigen::Matrix4d                 leftWrist2xrWorldPose;
        Eigen::Matrix4d                 rightWrist2xrWorldPose;

        // 双手的局部坐标系，包含25个点
        Eigen::Matrix<double,25,3>      leftHandPositions;
        Eigen::Matrix<double,25,3>      rightHandPositions;

        bool isLockHead;
    };

    Transform();

    ~Transform();

    virtual std::vector<Eigen::Matrix4d> Solve(const Transform::MsgConfig &config_) = 0;

    static boost::shared_ptr<Transform> GetPtr(const Transform::BasicConfig &config_);

    static Transform::Type GetTypeFromStr(const std::string& str);

protected:

    static const std::unordered_map<std::string, Transform::Type> typeMap;

};
