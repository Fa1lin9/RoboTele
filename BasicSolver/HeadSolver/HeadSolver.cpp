#include <HeadSolver/HeadSolver.hpp>

HeadSolver::HeadSolver(const Eigen::Matrix4d &mat){
    this->headPose = mat;

}

HeadSolver::~HeadSolver(){

}

