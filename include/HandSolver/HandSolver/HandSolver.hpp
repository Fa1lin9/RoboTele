#pragma once

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

#include <HandBase.hpp>

#include <XRBase.hpp>
class HandSolver
{
public:
    struct BasicConfig
    {
        XRBase::XRType type;
        int dofHand;
    };

    struct Data{
        std::vector<Eigen::Vector3d> leftHandPositions;
        std::vector<Eigen::Vector3d> rightHandPositions;
    };


    HandSolver();
    ~HandSolver();

    virtual Eigen::VectorXd Solve(const HandSolver::Data& data) = 0;


    static boost::shared_ptr<HandSolver> GetPtr(const HandSolver::BasicConfig& config_);

protected:

    int dofHand;
};
