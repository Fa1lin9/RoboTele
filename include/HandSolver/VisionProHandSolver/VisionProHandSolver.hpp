#pragma once

#include <HandSolver/HandSolver.hpp>


class VisionProHandSolver
        : public HandSolver
{
public:
    VisionProHandSolver(const HandSolver::BasicConfig &config_);
    ~VisionProHandSolver();

    Eigen::VectorXd Solve(const HandSolver::Data& data) override;


private:
    void Init();

    double CalVecAngle(const Eigen::Vector3d& origin,
                       const Eigen::Vector3d& point1,
                       const Eigen::Vector3d& );

    std::vector<Eigen::Vector3d> leftHandPositions;
    std::vector<Eigen::Vector3d> rightHandPositions;

    std::vector<size_t> thumbFingerJointsIndex;
    std::vector<size_t> indexFingerJointsIndex;
    std::vector<size_t> middleFingerJointsIndex;
    std::vector<size_t> ringFingerJointsIndex;
    std::vector<size_t> littleFingerJointsIndex;

};
