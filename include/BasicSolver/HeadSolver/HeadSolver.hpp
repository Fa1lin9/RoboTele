#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <boost/optional.hpp>

class HeadSolver
{
public:
    HeadSolver();
    HeadSolver(const Eigen::Matrix4d &mat);
    ~HeadSolver();

    void Init(const Eigen::Matrix4d &mat);

    inline Eigen::Vector3d GetValue(){
        if(this->rpy.has_value()){
            return this->rpy.value();
        }else{
            throw std::logic_error("[HeadSolver::GetValue] The rpy don't have value");
        }
    };
private:
    Eigen::Matrix4d headPose;
    boost::optional<Eigen::Vector3d> rpy;

    double roll;
    double pitch;
    double yaw;
};

