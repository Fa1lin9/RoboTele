#pragma once

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

#include <WeightedMovingFilter/WeightedMovingFilter.hpp>

#include <HandBase.hpp>

#include <XRBase.hpp>
class HandSolver
{
public:
    struct BasicConfig
    {
        XRBase::XRType type;
        int handDof;
    };

    HandSolver();
    ~HandSolver();

    virtual Eigen::VectorXd SolveDualHand(const HandBase::DualHandData& data) = 0;

    virtual Eigen::VectorXd SolveSingleHand(const HandBase::HandData& data,
                                            const HandBase::HandType& type) = 0;

    virtual std::vector<double> GetLowerBound();

    virtual std::vector<double> GetUpperBound();

    virtual std::vector<std::string> GetFingersName();

    static std::shared_ptr<HandSolver> GetPtr(const HandSolver::BasicConfig& config_);

protected:

    int handDof;

    std::vector<std::string> fingersName;
    std::vector<double> fingersUpperBound;
    std::vector<double> fingersLowerBound;

};
