#include <HeadSolver/HeadSolver.hpp>

HeadSolver::HeadSolver(){

}

HeadSolver::HeadSolver(const Eigen::Matrix4d &mat){

    this->Init(mat);
}

HeadSolver::~HeadSolver(){

}

void HeadSolver::Init(const Eigen::Matrix4d &mat){
    this->headPose = mat;

    this->rpy = this->headPose.block<3,3>(0,0).eulerAngles(2,1,0);

    this->roll = this->rpy.value()(2);
    this->pitch = this->rpy.value()(1);
    this->yaw = this->rpy.value()(0);
}
