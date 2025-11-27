#include <VisionProHandSolver/VisionProHandSolver.hpp>

VisionProHandSolver::VisionProHandSolver(const HandSolver::BasicConfig &config_){

    this->Init();
}

Eigen::VectorXd VisionProHandSolver::Solve(const HandSolver::Data& data){
    Eigen::VectorXd ret;
    this->leftHandPositions = data.leftHandPositions;
    this->rightHandPositions = data.rightHandPositions;

    // Index Finger
    double indexAngle =
            this->CalVecAngle(  this->leftHandPositions[this->indexFingerJointsIndex[1]],
                                this->leftHandPositions[this->indexFingerJointsIndex[0]],
                                this->leftHandPositions[this->indexFingerJointsIndex[2]]);


    return ret;
}

VisionProHandSolver::~VisionProHandSolver(){


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
