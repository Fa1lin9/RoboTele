#pragma once

#include <Transform/Transform.hpp>

// 实现坐标系从VisionPro到Ti5Robot的转换
class VisionProTransform
    : public Transform
{
public:
    VisionProTransform(const Transform::BasicConfig &config_);
    ~VisionProTransform();

    std::vector<Eigen::Matrix4d> Solve(const Transform::MsgConfig &config_) override;

private:

    Eigen::Matrix<double,4,4>                         T_Head2Waist;//头到机器人腰

    Eigen::Matrix<double,4,4>                         T_XR2Robot;//OpenXR到机器人（旋转角度）
    Eigen::Matrix<double,4,4>                         T_Robot2LeftWrist;//机器人基坐标系到手腕（旋转角度）
    Eigen::Matrix<double,4,4>                         T_Robot2RightWrist;//机器人基坐标系到手腕（旋转角度）

    Eigen::Vector3d                                   offset;

};
