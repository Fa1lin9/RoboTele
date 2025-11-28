#pragma once

#include <HandSolver/HandSolver.hpp>


class VisionProHandSolver
        : public HandSolver
{
public:
    VisionProHandSolver(const HandSolver::BasicConfig &config_);
    ~VisionProHandSolver();

    Eigen::VectorXd Solve(const HandSolver::Data& data) override;

    Eigen::VectorXd SolveSingleHand(const std::vector<Eigen::Vector3d> handPositions);

    Eigen::VectorXd SolveDualHand(const std::vector<Eigen::Vector3d> leftHandPositions_,
                                  const std::vector<Eigen::Vector3d> rightHandPositions_);

private:
    void Init();

    double CalVecAngle(const Eigen::Vector3d& origin,
                       const Eigen::Vector3d& point1,
                       const Eigen::Vector3d& point2);

    std::vector<Eigen::Vector3d> leftHandPositions;
    std::vector<Eigen::Vector3d> rightHandPositions;

    std::vector<size_t> thumbFingerJointsIndex;
    std::vector<size_t> indexFingerJointsIndex;
    std::vector<size_t> middleFingerJointsIndex;
    std::vector<size_t> ringFingerJointsIndex;
    std::vector<size_t> littleFingerJointsIndex;

};
