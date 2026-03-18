#pragma once

#include <iostream>

#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>


#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/autodiff/casadi.hpp>

#include <casadi/casadi.hpp>

#include <nlopt.hpp>

#include <RobotBase.hpp>
#include <MatrixUtils.hpp>
#include <FunctionLogger.hpp>

#include <JsonParser/JsonParser.hpp>

//#include <WeightedMovingFilter/WeightedMovingFilter.hpp>

class ArmSolver
{
public:
    enum Type{
        Ti5DualArm,
        G1Dof23DualArm,
        G1Dof29DualArm,
        GenericDualArm,
    };

    struct BasicConfig
    {
        bool useRootPath;
        std::string modelPath;

        ArmSolver::Type type;
        RobotBase::RobotType robotType;

        std::vector<std::string> baseFrameName;
        std::vector<std::string> targetFrameName;

        std::vector<Eigen::Matrix4d> baseOffset;
        std::vector<Eigen::Matrix4d> targetOffset;

        std::vector<std::string> leftArmJointNames;
        std::vector<std::string> rightArmJointNames;

        Eigen::VectorXd initPose;

        int maxIteration;
        double relativeTol;
        std::vector<int> armActiveDof;

        double wTranslation;
        double wRotation;
        double wRegularization;
        double wSmooth;
    };

    // just for nlopt
    // but currently we use the casadi to solve
    // so we don't use it anymore
    struct Ti5RobotConfig{

        Eigen::VectorXd q;
        Eigen::VectorXd qInit;
        // 2 pose: left arm and right arm
        std::vector<Eigen::Matrix4d> targetPose;
//        Eigen::Matrix4d leftArmTargetPose;
//        Eigen::Matrix4d rightArmTargetPose;
    };

    ArmSolver();
    ~ArmSolver();

    // Solve the IK
    virtual boost::optional<Eigen::VectorXd> Solve(
                    const std::vector<Eigen::Matrix4d>& targetPose,
                    const Eigen::VectorXd& qInit,
                    bool verbose) = 0;

    virtual std::vector<pinocchio::SE3> Forward(const Eigen::VectorXd& q) = 0;

    // Output some information of the current solver
    virtual void Info() = 0;

    virtual size_t GetTotalDof() = 0;

    virtual std::vector<std::string> GetJointNames() = 0;

    virtual std::vector<std::string> GetLeftArmJointNames();

    virtual std::vector<std::string> GetRightArmJointNames();

    virtual std::vector<size_t> GetLeftArmJointIndex();

    virtual std::vector<size_t> GetRightArmJointIndex();

    virtual std::vector<size_t> GetLeftArmQIndex();

    virtual std::vector<size_t> GetRightArmQIndex();

    static std::shared_ptr<ArmSolver> GetPtr(const ArmSolver::BasicConfig& config_);

    static std::shared_ptr<ArmSolver> GetPtr(const std::string& filePath);

    static ArmSolver::Type GetTypeFromStr(const std::string& str);

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



private:
    static const std::unordered_map<std::string, ArmSolver::Type> typeMap;
};
