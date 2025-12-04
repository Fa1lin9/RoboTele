#pragma once
#include <Eigen/Dense>
#include <iostream>

namespace MatrixUtils{
    enum EulerAxis{
        Roll,
        Pitch,
        Yaw,
    };

    const std::unordered_map<std::string, MatrixUtils::EulerAxis> EulerAxisMap = {
        {"Roll", MatrixUtils::EulerAxis::Roll},
        {"Pitch", MatrixUtils::EulerAxis::Pitch},
        {"Yaw", MatrixUtils::EulerAxis::Yaw}
    };

    static MatrixUtils::EulerAxis GetEulerAxisFromStr(const std::string& str){
        auto temp = MatrixUtils::EulerAxisMap.find(str);
        if(temp != MatrixUtils::EulerAxisMap.end()){
            return temp->second;
        }

        throw std::invalid_argument("[MatrixUtils::GetEulerAxisFromStr] Invalid string");
    }

    static std::string GetStrFromEulerAxis(const MatrixUtils::EulerAxis& type){
        for(const auto& kv : MatrixUtils::EulerAxisMap){
            if(kv.second == type){
                return kv.first;
            }
        }
        throw std::invalid_argument("[MatrixUtils::GetStrFromType] Invalid type");
    }


    // Calculate the Euler Angle
    inline Eigen::Vector3d RotationToEulerXYZ(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d rpy;

        double sy = -R(0,2);
        if (sy > 1.0) sy = 1.0;
        if (sy < -1.0) sy = -1.0;

        rpy[1] = std::asin(sy);  // Y

        if (std::abs(std::cos(rpy[1])) > 1e-6)
        {
            rpy[0] = std::atan2(R(1,2), R(2,2));  // X
            rpy[2] = std::atan2(R(0,1), R(0,0));  // Z
        }
        else
        {
            rpy[0] = 0.0;
            rpy[2] = std::atan2(-R(1,0), R(1,1));
        }

        return rpy;
    }

    inline Eigen::Vector3d RotationToEulerZYX(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d rpy;

        // pitch = asin(-R(2,0))
        double sy = -R(2,0);
        if (sy > 1.0) sy = 1.0;
        if (sy < -1.0) sy = -1.0;

        rpy[1] = std::asin(sy);  // pitch

        // Check singularity: |cos(pitch)| < small
        if (std::abs(std::cos(rpy[1])) > 1e-6)
        {
            rpy[0] = std::atan2(R(2,1), R(2,2));  // roll
            rpy[2] = std::atan2(R(1,0), R(0,0));  // yaw
        }
        else
        {
            // Gimbal lock: pitch = +-90°
            rpy[0] = 0.0;
            rpy[2] = std::atan2(-R(0,1), R(1,1));
        }

        return rpy;  // [roll, pitch, yaw]
    }

    inline Eigen::Vector3d RotationToEulerXZY(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d eul;  // [roll(X), pitch(Y), yaw(Z)]

        // pitch = atan2(R(0,2), R(0,0))
        double sp = std::atan2(R(0,2), R(0,0));  // pitch (beta)
        // clamp not needed for atan2 result, but keep variable
        eul[1] = sp;  // pitch -> eul[1]

        double cp = std::cos(eul[1]);

        if (std::abs(cp) > 1e-6)
        {
            // 非奇异
            // yaw (gamma): 从 R(0,1) = -sin(gamma) 和 R(0,0) = cos(gamma)*cos(beta)
            // 得到 cos(gamma) = R(0,0)/cos(beta)
            double yaw = std::atan2(-R(0,1), R(0,0) / cp);
            // roll (alpha): 从 R(2,1) = sin(alpha)*cos(gamma), R(1,1) = cos(alpha)*cos(gamma)
            double roll = std::atan2(R(2,1), R(1,1));

            eul[0] = roll;
            eul[2] = yaw;
        }
        else
        {
            // Gimbal lock (cos(pitch) ≈ 0), pitch = ±90°
            // 在这种情况下，cos(yaw) 因为乘以 cos(beta) 在很多元素中丢失，
            // roll 和 yaw 会耦合。按照常见做法固定一个角为 0（这里设 roll = 0）
            // 并用剩余的矩阵元解出 yaw（或用其它一致策略）。
            eul[0] = 0.0;  // roll = 0 (约定)
            // 选择用 R(1,2) 和 R(2,2) 来恢复 yaw（见上文推导的特殊情形）
            eul[2] = std::atan2(R(1,2), R(2,2)); // yaw
            // pitch 已经赋给 eul[1]
        }

        return eul; // [roll, pitch, yaw]
    }


    inline void WrapAngleToPi(Eigen::VectorXd& angle){
        for(int i=0;i<angle.size();i++){
            angle(i) = fmod(angle(i) + M_PI, 2 * M_PI); // 先 +π 再取模
            if (angle(i) < 0) {
                angle(i) += 2 * M_PI; // 确保在 [0, 2π]
            }
            angle(i) -= M_PI; // 回到 [-π, π]
        }
    }

    // Check the Pose Matrix
    inline bool IsPoseMatrix(const Eigen::Matrix4d &mat,
                      const double& eps = 1e-2){
        Eigen::Matrix3d rotation = mat.block<3,3>(0,0);
        if(!(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity() , eps)){
            std::cout<<"rotation.transpose() * rotation: \n"<<rotation.transpose() * rotation<<std::endl;
            return false;
        }

        if(std::abs(mat.determinant()-1) > eps){
            std::cout<<"mat.determinant()-1: "<<mat.determinant()-1<<std::endl;
            return false;
        }

        if(mat(3,0) != 0.0 || mat(3,1) != 0.0 || mat(3,2) != 0.0 || mat(3,3) != 1.0){
            return false;
        }

        return true;
    }

    // Calculate the Distance between Two Pose Matrix
    inline double CalTransDist(const Eigen::Matrix4d& mat1,
                               const Eigen::Matrix4d& mat2){
        return (mat1.block<3,1>(0,3) - mat2.block<3,1>(0,3)).norm();
    }

}
