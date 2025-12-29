#pragma once
#include <ArmSolver/ArmSolver.hpp>
#include <source_path.h>
#include <iostream>
#include <string>
#include <chrono>

/* ------------------ Details of the Joints ------------------

Joint 1: left_hip_pitch_joint, type: JointModelRY, parent: 0
Joint 2: left_hip_roll_joint, type: JointModelRX, parent: 1
Joint 3: left_hip_yaw_joint, type: JointModelRZ, parent: 2
Joint 4: left_knee_joint, type: JointModelRY, parent: 3
Joint 5: left_ankle_pitch_joint, type: JointModelRY, parent: 4
Joint 6: left_ankle_roll_joint, type: JointModelRX, parent: 5
Joint 7: right_hip_pitch_joint, type: JointModelRY, parent: 0
Joint 8: right_hip_roll_joint, type: JointModelRX, parent: 7
Joint 9: right_hip_yaw_joint, type: JointModelRZ, parent: 8
Joint 10: right_knee_joint, type: JointModelRY, parent: 9
Joint 11: right_ankle_pitch_joint, type: JointModelRY, parent: 10
Joint 12: right_ankle_roll_joint, type: JointModelRX, parent: 11
Joint 13: waist_yaw_joint, type: JointModelRZ, parent: 0
Joint 14: waist_roll_joint, type: JointModelRX, parent: 13
Joint 15: waist_pitch_joint, type: JointModelRY, parent: 14
Joint 16: left_shoulder_pitch_joint, type: JointModelRY, parent: 15
Joint 17: left_shoulder_roll_joint, type: JointModelRX, parent: 16
Joint 18: left_shoulder_yaw_joint, type: JointModelRZ, parent: 17
Joint 19: left_elbow_joint, type: JointModelRY, parent: 18
Joint 20: left_wrist_roll_joint, type: JointModelRX, parent: 19
Joint 21: left_wrist_pitch_joint, type: JointModelRY, parent: 20
Joint 22: left_wrist_yaw_joint, type: JointModelRZ, parent: 21
Joint 23: left_hand_index_0_joint, type: JointModelRZ, parent: 22
Joint 24: left_hand_index_1_joint, type: JointModelRZ, parent: 23
Joint 25: left_hand_middle_0_joint, type: JointModelRZ, parent: 22
Joint 26: left_hand_middle_1_joint, type: JointModelRZ, parent: 25
Joint 27: left_hand_thumb_0_joint, type: JointModelRY, parent: 22
Joint 28: left_hand_thumb_1_joint, type: JointModelRZ, parent: 27
Joint 29: left_hand_thumb_2_joint, type: JointModelRZ, parent: 28
Joint 30: right_shoulder_pitch_joint, type: JointModelRY, parent: 15
Joint 31: right_shoulder_roll_joint, type: JointModelRX, parent: 30
Joint 32: right_shoulder_yaw_joint, type: JointModelRZ, parent: 31
Joint 33: right_elbow_joint, type: JointModelRY, parent: 32
Joint 34: right_wrist_roll_joint, type: JointModelRX, parent: 33
Joint 35: right_wrist_pitch_joint, type: JointModelRY, parent: 34
Joint 36: right_wrist_yaw_joint, type: JointModelRZ, parent: 35
Joint 37: right_hand_index_0_joint, type: JointModelRZ, parent: 36
Joint 38: right_hand_index_1_joint, type: JointModelRZ, parent: 37
Joint 39: right_hand_middle_0_joint, type: JointModelRZ, parent: 36
Joint 40: right_hand_middle_1_joint, type: JointModelRZ, parent: 39
Joint 41: right_hand_thumb_0_joint, type: JointModelRY, parent: 36
Joint 42: right_hand_thumb_1_joint, type: JointModelRZ, parent: 41
Joint 43: right_hand_thumb_2_joint, type: JointModelRZ, parent: 42

 ------------------ Details of the Joints ------------------ */

class G1Dof29DualArmSolver
        :public ArmSolver
{
public:
    G1Dof29DualArmSolver(const ArmSolver::BasicConfig &config_);
    ~G1Dof29DualArmSolver();

    boost::optional<Eigen::VectorXd> Solve(
                    const std::vector<Eigen::Matrix4d>& targetPose,
                    const Eigen::VectorXd& qLast_,
                    bool verbose) override;

    std::vector<pinocchio::SE3> Forward(const Eigen::VectorXd& q) override;

    size_t GetTotalDof() override;

    std::vector<std::string> GetJointNames() override;

    void Info() override;

private:
    /* ------------------ Basic Info ------------------ */
    // the degree of freedom of Unitree G1
    // 7 * 2 + 6 * 2 + 3 + 7 * 2 = 14 + 12 + 3 + 14 = 43
    // No dof of head
    const size_t totalDof = 43;

    std::vector<std::string> jointNames;

    const size_t armDof = 7;

    const size_t handDof = 7;

    const size_t waistDof = 3;

    const size_t legDof = 6;

    // Initialization
    void InitRobot(const ArmSolver::BasicConfig &config_);

    void InitOptim(const ArmSolver::BasicConfig &config_);

    // the urdf file path of the Unitree G1
    const std::string modelPath =
            std::string(SOURCE_FILE_PATH) + "/assets/urdf/UnitreeG1/g1_body29_hand14.urdf";

    /* ------------------ Robot Parameter ------------------ */

    pinocchio::Model robotModel;

    std::vector<size_t> leftArmID = {16, 17, 18, 19, 20, 21, 22};

    std::vector<size_t> rightArmID = {30, 31, 32, 33, 34, 35, 36};

    size_t baseIndex;
    size_t leftArmEndIndex;
    size_t rightArmEndIndex;

    std::vector<double> leftArmLowerBound;
    std::vector<double> rightArmLowerBound;

    std::vector<double> leftArmUpperBound;
    std::vector<double> rightArmUpperBound;

    std::vector<double> totalLowerBound;
    std::vector<double> totalUpperBound;

    Eigen::VectorXd initPose;

    /* ------------------ Casadi Auto-Diff ------------------ */

    pinocchio::ModelTpl<casadi::SX> robotModelSX;

    Eigen::Matrix4d baseOffset;
    Eigen::Matrix<casadi::SX,4,4> baseOffsetSX;

    // Opti
    std::shared_ptr<pinocchio::DataTpl<casadi::SX>> dataPtrSX;

    casadi::Opti opti;
    casadi::MX qVar;
    casadi::MX qLast;
    casadi::MX targetPoseLeft;
    casadi::MX targetPoseRight;

    casadi::Function translationalError;
    casadi::Function rotationalError;

    casadi::MX translationalCost;
    casadi::MX rotationalCost;
    casadi::MX smoothCost;
    casadi::MX regularizationCost;
    casadi::MX totalCost;

    /* ------------------ NLopt ------------------ */
    double relativeTol = 1e-3;
    size_t maxIteration = 400;

    double wTranslation;
    double wRotation;
    double wRegularization;
    double wSmooth;

};
