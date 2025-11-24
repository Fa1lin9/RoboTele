#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>
#include <iostream>
#include <FunctionLogger.hpp>

#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/autodiff/casadi.hpp>

#include <casadi/casadi.hpp>

#include <nlopt.hpp>

#include <RobotType.hpp>

//#include <WeightedMovingFilter/WeightedMovingFilter.hpp>

class IKSolver
{
public:
//    enum Type{
//        Ti5Robot
//    };

    struct BasicConfig
    {
        std::string modelPath;

        RobotType::Type robotType;

        std::vector<std::string> baseFrameName;
        std::vector<std::string> targetFrameName;

        std::vector<Eigen::Matrix4d> baseOffset;
        std::vector<Eigen::Matrix4d> targetOffset;

        int maxIteration;
        double relativeTol;
        int dofLeftArm;
        int dofRightArm;
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

    IKSolver();
    ~IKSolver();

    // Solver the IK
    //考虑到目标位姿包含双臂末端，同时考虑到泛化性，所以用std::vector
    virtual boost::optional<Eigen::VectorXd> Solve(
                    const std::vector<Eigen::Matrix4d>& targetPose,
                    const Eigen::VectorXd& qInit,
                    bool verbose) = 0;

    virtual std::vector<pinocchio::SE3> Forward(const Eigen::VectorXd& q) = 0;

    // Output some information of the current solver
    virtual void Info() = 0;

    virtual size_t GetDofTotal() = 0;

    static boost::shared_ptr<IKSolver> GetPtr(const IKSolver::BasicConfig& config_);

private:

};
