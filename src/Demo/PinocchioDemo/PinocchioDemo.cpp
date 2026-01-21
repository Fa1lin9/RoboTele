#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <source_path.h>

#include <ArmSolver/ArmSolver.hpp>

const std::string modelPath =
        std::string(SOURCE_FILE_PATH)+"/assets/urdf/update_kanuopu-robot.urdf";

int main(){
    // BaseOffset
    Eigen::Matrix4d baseOffset;
    baseOffset << 1, 0, 0, +0.02,
                    0, 1, 0, 0,
                    0, 0, 1, +1.10,
                    0, 0 ,0, 1;

    // TargetOffset
    Eigen::Matrix4d targetOffset;
    targetOffset << 1, 0, 0, +0.2,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0 ,0, 1;

    ArmSolver::BasicConfig config = {
        .type = ArmSolver::Type::G1Dof23DualArm,
        .baseFrameName = {"waist_yaw_joint"},
        .targetFrameName = {"left_wrist_roll_joint", "right_wrist_roll_joint"},
        .baseOffset = std::vector<Eigen::Matrix4d>{baseOffset},
        .targetOffset = std::vector<Eigen::Matrix4d>{targetOffset, targetOffset},
        .maxIteration = 50,
        .relativeTol = 1e-6,
        .armActiveDof = {5, 5},
        .wTranslation = 50.0,
        .wRotation = 0.5,
        .wRegularization = 0.02,
        .wSmooth = 0.5,
    };

    std::shared_ptr<ArmSolver> ikSolverPtr = ArmSolver::GetPtr(config);

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
