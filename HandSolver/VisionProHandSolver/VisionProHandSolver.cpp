#include <VisionProHandSolver/VisionProHandSolver.hpp>

VisionProHandSolver::VisionProHandSolver(const HandSolver::BasicConfig &config_){
    this->dofHand = config_.dofHand;
    this->Init();
}

VisionProHandSolver::~VisionProHandSolver(){


}

Eigen::VectorXd VisionProHandSolver::Solve(const HandSolver::Data& data){
    return this->SolveDualHand(data.leftHandPositions,
                               data.rightHandPositions);
}

Eigen::VectorXd VisionProHandSolver::SolveSingleHand(const std::vector<Eigen::Vector3d> handPositions){
    std::vector<double> ret;
    auto data = handPositions;

    // Thumb Finger
    double thumbAngle =
            this->CalVecAngle(  data[this->thumbFingerJointsIndex[1]],
                                data[this->thumbFingerJointsIndex[2]],
                                data[this->thumbFingerJointsIndex[3]]);
    ret.push_back(thumbAngle);

    // Index Finger
    double indexAngle =
            this->CalVecAngle(  data[this->indexFingerJointsIndex[1]],
                                data[this->indexFingerJointsIndex[0]],
                                data[this->indexFingerJointsIndex[2]]);
    ret.push_back(indexAngle);

    // Middle Finger
    double middleAngle =
            this->CalVecAngle(  data[this->middleFingerJointsIndex[1]],
                                data[this->middleFingerJointsIndex[0]],
                                data[this->middleFingerJointsIndex[2]]);
    ret.push_back(middleAngle);

    // Ring Finger
    double ringAngle =
            this->CalVecAngle(  data[this->ringFingerJointsIndex[1]],
                                data[this->ringFingerJointsIndex[0]],
                                data[this->ringFingerJointsIndex[2]]);
    ret.push_back(ringAngle);

    // Little Finger
    double littleAngle =
            this->CalVecAngle(  data[this->littleFingerJointsIndex[1]],
                                data[this->littleFingerJointsIndex[0]],
                                data[this->littleFingerJointsIndex[2]]);
    ret.push_back(littleAngle);

    // Thumb Finger Rotation
    double thumbRotAngle =
            this->CalVecAngle(  data[this->indexFingerJointsIndex[1]], // 6
                                data[this->thumbFingerJointsIndex[2]], // 3
                                data[this->littleFingerJointsIndex[1]]); // 21
    thumbRotAngle = 180.0 - thumbRotAngle;
    ret.push_back(thumbRotAngle);

    return Eigen::VectorXd::Map(ret.data(), ret.size());
}

Eigen::VectorXd VisionProHandSolver::SolveDualHand(const std::vector<Eigen::Vector3d> leftHandPositions_,
                                                   const std::vector<Eigen::Vector3d> rightHandPositions_)
{
    auto leftHandAngle = this->SolveSingleHand(leftHandPositions_);
    auto rightHandAngle = this->SolveSingleHand(rightHandPositions_);

    Eigen::VectorXd ret(leftHandAngle.size() + rightHandAngle.size());
    ret << leftHandAngle, rightHandAngle;

    return ret;
}

double VisionProHandSolver::CalVecAngle(const Eigen::Vector3d& origin,
                                        const Eigen::Vector3d& point1,
                                        const Eigen::Vector3d& point2){
    Eigen::Vector3d vec1 = point1 - origin;
    Eigen::Vector3d vec2 = point2 - origin;

    double cos = vec1.normalized().dot(vec2.normalized());
    cos = std::clamp(cos, -1.0, 1.0);

    return std::acos(cos) * 180.0 / M_PI;
}

void VisionProHandSolver::Init(){
    this->thumbFingerJointsIndex = {1, 2, 3, 4};
    this->indexFingerJointsIndex = {5 ,6, 7, 8, 9};
    this->middleFingerJointsIndex = {10, 11, 12, 13, 14};
    this->ringFingerJointsIndex = {15, 16, 17, 18, 19};
    this->littleFingerJointsIndex = {20, 21, 22, 23, 24};
}
