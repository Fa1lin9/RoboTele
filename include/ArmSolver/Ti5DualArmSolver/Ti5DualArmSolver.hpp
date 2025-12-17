#pragma once
#include <ArmSolver/ArmSolver.hpp>
#include <source_path.h>

#include <chrono>

/* ------------------ Details of the Joints ------------------

Joint 1: UP_DOWN, type: JointModelPZ, parent: 0
Joint 2: WAIST_R, type: JointModelRX, parent: 1
Joint 3: WAIST_Y, type: JointModelRZ, parent: 2
Joint 4: WAIST_P, type: JointModelRevoluteUnaligned, parent: 3
Joint 5: L1_SHOULDER_P, type: JointModelRY, parent: 4
Joint 6: L2_SHOULDER_R, type: JointModelRX, parent: 5
Joint 7: L3_SHOULDER_Y, type: JointModelRY, parent: 6
Joint 8: L4_ELBOW_Y, type: JointModelRX, parent: 7
Joint 9: L5_WRIST_P, type: JointModelRY, parent: 8
Joint 10: L6_WRIST_Y, type: JointModelRZ, parent: 9
Joint 11: L7_WRIST_R, type: JointModelRX, parent: 10
Joint 12: NECK_Y, type: JointModelRZ, parent: 4
Joint 13: NECK_P, type: JointModelRevoluteUnaligned, parent: 12
Joint 14: NECK_R, type: JointModelRX, parent: 13
Joint 15: R1_SHOULDER_P, type: JointModelRevoluteUnaligned, parent: 4
Joint 16: R2_SHOULDER_R, type: JointModelRX, parent: 15
Joint 17: R3_SHOULDER_Y, type: JointModelRevoluteUnaligned, parent: 16
Joint 18: R4_ELBOW_Y, type: JointModelRX, parent: 17
Joint 19: R5_WRIST_P, type: JointModelRevoluteUnaligned, parent: 18
Joint 20: R6_WRIST_Y, type: JointModelRZ, parent: 19
Joint 21: R7_WRIST_R, type: JointModelRX, parent: 20

 ------------------ Details of the Joints ------------------ */

class Ti5DualArmSolver
        :public ArmSolver
{
public:
    enum SolverType{
        Casadi,
        Nlopt,
    };

    Ti5DualArmSolver(const ArmSolver::BasicConfig &config_);
    ~Ti5DualArmSolver();

    boost::optional<Eigen::VectorXd> Solve(
                    const std::vector<Eigen::Matrix4d>& targetPose,
                    const Eigen::VectorXd& qLast_,
                    bool verbose) override;

    std::vector<pinocchio::SE3> Forward(const Eigen::VectorXd& q) override;

    size_t GetDofTotal() override;

    std::vector<std::string> GetJointNames() override;

    void Info() override;

private:
    /* ------------------ Basic Info ------------------ */
    // the degree of freedom of CRP's Robot
    // 一只手7个自由度
    // 头部3个自由度
    // 腰部3个自由度
    // AGV处有个UP_DOWN关节，是沿着Z轴的平移关节
    // 总共 7 * 2 + 3 + 3 + 1 = 21
    const size_t dofTotal = 21;

    std::vector<std::string> jointNames;

    const size_t dofArm = 7;

    struct Ti5RobotData{
        Ti5DualArmSolver *solver;
        Eigen::VectorXd qInit;
        std::vector<Eigen::Matrix4d> targetPose;
//        Eigen::Matrix4d leftArmTargetPose;
//        Eigen::Matrix4d rightArmTargetPose;
    };

private:
    double ObjectiveFunc(const ArmSolver::Ti5RobotConfig& config_);

    casadi::SX ObjectiveFuncSX(const pinocchio::ModelTpl<casadi::SX>::ConfigVectorType& q,
                        const Eigen::Matrix<casadi::SX,Eigen::Dynamic,1>& qInit,
                        const std::vector<Eigen::Matrix<casadi::SX,4,4>>& targetPose
                        );

    // Initialization
    void InitRobot(const ArmSolver::BasicConfig &config_);

    void InitOptim(const ArmSolver::BasicConfig &config_);

    void InitAD(const std::vector<Eigen::Matrix4d>& targetPose,
                      const Eigen::VectorXd& qInit);

    template<int Rows, int Cols>
    casadi::SX Eigen2SX(const Eigen::Matrix<casadi::SX, Rows, Cols>& mat) const {
        casadi::SX result = casadi::SX::zeros(Rows, Cols);
        for(int i=0;i<Rows;i++){
            for(int j=0;j<Cols;j++){
                result(i,j) = mat(i,j);
            }
        }
        return result;
    }

    Eigen::Matrix<casadi::SX, Eigen::Dynamic, Eigen::Dynamic> SX2Eigen(const casadi::SX& mat) const {
        int rows = mat.size1();
        int cols = mat.size2();
        Eigen::Matrix<casadi::SX, Eigen::Dynamic, Eigen::Dynamic> result(rows, cols);
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                result(i, j) = mat(i,j);
        return result;
    }


    // the urdf file path of the CRP's Robot
    const std::string modelPath =
            std::string(SOURCE_FILE_PATH) + "/assets/urdf/Ti5Robot/Ti5Robot.urdf";

    /* ------------------ Robot Parameter ------------------ */

    pinocchio::Model robotModel;

    std::vector<size_t> leftArmID = {5,6,7,8,9,10,11};

    std::vector<size_t> rightArmID = {15,16,17,18,19,20,21};

    size_t baseIndex;
    size_t leftArmEndIndex;
    size_t rightArmEndIndex;

    std::vector<double> qLeftArmNeutral;
    std::vector<double> qRightArmNeutral;
    std::vector<double> qNeutral;

    std::vector<double> leftArmLowerBound;
    std::vector<double> rightArmLowerBound;

    std::vector<double> leftArmUpperBound;
    std::vector<double> rightArmUpperBound;

    std::vector<double> totalLowerBound;
    std::vector<double> totalUpperBound;

    Eigen::VectorXd initPose;

    /* ------------------ Casadi Auto-Diff ------------------ */

    pinocchio::ModelTpl<casadi::SX> robotModelSX;

    // Function
    casadi::Function mainFunc;

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
