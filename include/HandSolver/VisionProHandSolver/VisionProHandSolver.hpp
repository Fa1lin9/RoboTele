#pragma once

#include <HandSolver/HandSolver.hpp>
#include <MatrixUtils.hpp>

class VisionProHandSolver
        : public HandSolver
{
public:
    VisionProHandSolver(const HandSolver::BasicConfig &config_);
    ~VisionProHandSolver();

    Eigen::VectorXd SolveSingleHand(const HandBase::HandData& data,
                                    const HandBase::HandType& type) override;

    Eigen::VectorXd SolveDualHand(const HandBase::DualHandData& data) override;

private:
    void Init();

    Eigen::VectorXd MapXR2Hand(const std::vector<double>& angles,
                        const HandBase::HandType& type);

    std::vector<Eigen::Vector3d> leftHandPositions;
    std::vector<Eigen::Vector3d> rightHandPositions;

    std::vector<size_t> thumbFingerJointsIndex;
    std::vector<size_t> indexFingerJointsIndex;
    std::vector<size_t> middleFingerJointsIndex;
    std::vector<size_t> ringFingerJointsIndex;
    std::vector<size_t> littleFingerJointsIndex;

    // Filter
    WeightedMovingFilter filter;

};


