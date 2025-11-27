#pragma once

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

class HandSolver
{
public:
    enum Type{
        VisionPro,

    };

    struct BasicConfig
    {
        HandSolver::Type type;
    };

    struct Data{
        std::vector<Eigen::Vector3d> leftHandPositions;
        std::vector<Eigen::Vector3d> rightHandPositions;
    };


    HandSolver();
    ~HandSolver();

    virtual Eigen::VectorXd Solve(const HandSolver::Data& data) = 0;


    static boost::shared_ptr<HandSolver> GetPtr(const HandSolver::BasicConfig& config_);

    static HandSolver::Type GetTypeFromStr(const std::string& str);


private:
    static const std::unordered_map<std::string, HandSolver::Type> typeMap;

};
