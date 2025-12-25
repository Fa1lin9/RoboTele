#pragma once

#include <Transform/Transform.hpp>

// 实现坐标系从VisionPro到UnitreeG1的转换
class VisionPro2UnitreeG1Transform
    : public Transform
{
public:
    VisionPro2UnitreeG1Transform(const Transform::BasicConfig &config_);
    ~VisionPro2UnitreeG1Transform();

    std::vector<Eigen::Matrix4d> Solve(const Transform::MsgConfig &config_) override;

private:

    Eigen::Matrix<double,4,4>                         T_Head2Waist;//头到机器人腰

    Eigen::Matrix<double,4,4>                         T_XR2Robot;//OpenXR到机器人（旋转角度）
    Eigen::Matrix<double,4,4>                         T_Robot2LeftWrist;//机器人基坐标系到手腕（旋转角度）
    Eigen::Matrix<double,4,4>                         T_Robot2RightWrist;//机器人基坐标系到手腕（旋转角度）

    Eigen::Vector3d                                   offset;

};
