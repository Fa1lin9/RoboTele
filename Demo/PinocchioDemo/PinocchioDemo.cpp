#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <source_path.h>

#include <ArmSolver/ArmSolver.hpp>

const std::string modelPath =
        std::string(SOURCE_FILE_PATH)+"/assets/urdf/update_kanuopu-robot.urdf";

int main(){
    ArmSolver::BasicConfig config = {
        .type = ArmSolver::Type::G1Dof29DualArm,
        .baseFrameName = {"waist_yaw_joint"},
        .targetFrameName = {"left_wrist_yaw_joint", "right_wrist_yaw_joint"},
        .maxIteration = 400,
        .relativeTol = 1e-4,
        .dofArm = {5, 5},
    };

    boost::shared_ptr<ArmSolver> ikSolverPtr = ArmSolver::GetPtr(config);

    ikSolverPtr->Info();

//    Eigen::VectorXd qInit = Eigen::VectorXd::Zero(21);
//    qInit.segment(4,7) << -0.72, -1.0, 0.57, -1.0, 0.83, 0, 0;
//    qInit.segment(14,7) << 0.72, 1.0, -0.57, 1.0, -0.83, 0, 0;
////    std::cout<<" qInit \n"<<qInit<<std::endl;

//    Eigen::Matrix4d leftArmTargetPose,rightArmTargetPose;
//    leftArmTargetPose <<     0 , 1 , 0 , 0.4,
//                             -1 , 0 , 0 , 0.1,
//                             0 , 0 , 1 , 0.0,
//                             0 , 0 , 0 , 1;
//    rightArmTargetPose <<    0 , -1 , 0 , 0.4,
//                             1 , 0 , 0 , -0.1,
//                             0 , 0 , 1 , 0.0,
//                             0 , 0 , 0 , 1;
//    std::vector<Eigen::Matrix4d> targetPose = {leftArmTargetPose, rightArmTargetPose};
//    std::cout<<" start to solve "<<std::endl;
//    ikSolverPtr->Solve(targetPose,qInit,true);
//    Eigen::VectorXd tempQ = Eigen::VectorXd::Zero(21);
//    tempQ.segment(14,2) << 1.507, 1.507;
//    tempQ << 0,0,0,0,1.50819,0.523597,-3.11438,-1.84003,-1.01497,0.0999991,-1.5533,0,0,0,-1.53059,-0.523597,3.11541,1.84045,0.998074,-0.0999991,1.5533;

//    // check result
//    std::cout<<"------------ Solver Result ------------"<<std::endl;
//    std::cout<<" Left Arm Translation: \n"<<ikSolverPtr->Forward(tempQ)[0].translation()<<std::endl;
//    std::cout<<" Left Arm Rotation: \n"<<ikSolverPtr->Forward(tempQ)[0].rotation()<<std::endl;
//    std::cout<<" Right Arm Translation: \n"<<ikSolverPtr->Forward(tempQ)[1].translation()<<std::endl;
//    std::cout<<" Rigit Arm Rotation: \n"<<ikSolverPtr->Forward(tempQ)[1].rotation()<<std::endl;
//    std::cout<<"------------ Solver Result ------------"<<std::endl;
}
