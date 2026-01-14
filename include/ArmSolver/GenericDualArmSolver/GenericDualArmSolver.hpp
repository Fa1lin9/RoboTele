#pragma once

#include <ArmSolver/ArmSolver.hpp>
#include <source_path.h>
#include <iostream>
#include <string>
#include <chrono>

class GenericDualArmSolver
        :public ArmSolver
{
public:
    GenericDualArmSolver(const ArmSolver::BasicConfig &config);
    ~GenericDualArmSolver();

    boost::optional<Eigen::VectorXd> Solve(
                    const std::vector<Eigen::Matrix4d>& targetPose,
                    const Eigen::VectorXd& qLast_,
                    bool verbose) override;

    std::vector<pinocchio::SE3> Forward(const Eigen::VectorXd& q) override;

    size_t GetTotalDof() override;

    std::vector<std::string> GetJointNames() override;

    std::vector<std::string> GetLeftArmJointNames() override;

    std::vector<std::string> GetRightArmJointNames() override;

    std::vector<size_t> GetLeftArmJointIndex() override;

    std::vector<size_t> GetRightArmJointIndex() override;

    std::vector<size_t> GetLeftArmQIndex() override;

    std::vector<size_t> GetRightArmQIndex() override;

    void Info() override;

private:
    // Initialization
    void InitRobot();

    void InitOptim();

    void CheckConfig(const ArmSolver::BasicConfig &config);

    void LoadConfig(const ArmSolver::BasicConfig &config);

    // Variable
    // Robot
    std::string modelPath;
    pinocchio::Model robotModel;
    size_t totalDof;
    size_t armDof;
    std::vector<int> armActiveDof;

    // Basic Info
    std::vector<std::string> jointNames;
    RobotBase::RobotType robotType;
    Eigen::VectorXd initPose;
    std::vector<Eigen::Matrix4d> baseOffset;
    std::vector<Eigen::Matrix4d> targetOffset;

    std::vector<std::string> baseFrameName;
    std::vector<std::string> targetFrameName;

    size_t baseFrameIndex;
    size_t leftArmEndJointIndex;
    size_t rightArmEndJointIndex;
    size_t leftArmEndEffectorFrameIndex;
    size_t rightArmEndEffectorFrameIndex;

    std::vector<std::string> leftArmJointNames;
    std::vector<std::string> rightArmJointNames;

    std::vector<size_t> leftArmJointIndex;
    std::vector<size_t> rightArmJointIndex;

    std::vector<size_t> leftArmQIndex;
    std::vector<size_t> rightArmQIndex;

    // Bound
    std::vector<double> leftArmLowerBound;
    std::vector<double> rightArmLowerBound;

    std::vector<double> leftArmUpperBound;
    std::vector<double> rightArmUpperBound;

    std::vector<double> totalLowerBound;
    std::vector<double> totalUpperBound;

    // Casadi
    pinocchio::ModelTpl<casadi::SX> robotModelSX;
    std::shared_ptr<pinocchio::DataTpl<casadi::SX>> dataPtrSX;
    Eigen::Matrix<casadi::SX,4,4> baseOffsetSX;

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

    // Optimization
    double wTranslation;
    double wRotation;
    double wRegularization;
    double wSmooth;

    double relativeTol;
    size_t maxIteration;


};
