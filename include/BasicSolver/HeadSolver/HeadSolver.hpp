#pragma once

#include <Eigen/Dense>
#include <iostream>

class HeadSolver
{
public:
    HeadSolver(const Eigen::Matrix4d &mat);
    ~HeadSolver();


private:
    Eigen::Matrix4d headPose;

};

