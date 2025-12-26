#pragma once
#include <iostream>
#include <unordered_map>
#include <MatrixUtils.hpp>

namespace RobotBase{

    struct JointInfo {
        std::string name;
        int index;
        MatrixUtils::EulerAxis type;
    };

    enum RobotType{
        Ti5Robot,
        UnitreeG1Dof29,
        UnitreeG1Dof23,
    };

    inline const std::unordered_map<std::string, RobotBase::RobotType> RobotTypeMap = {
        {"Ti5Robot", RobotBase::RobotType::Ti5Robot},
        {"UnitreeG1Dof29", RobotBase::RobotType::UnitreeG1Dof29},
        {"UnitreeG1Dof23", RobotBase::RobotType::UnitreeG1Dof23},
    };

    inline RobotBase::RobotType GetTypeFromStr(const std::string& str){
        auto temp = RobotBase::RobotTypeMap.find(str);
        if(temp != RobotBase::RobotTypeMap.end()){
            return temp->second;
        }

        throw std::invalid_argument("[RobotBase::RobotType::GetTypeFromStr] Invalid string");
    }

    inline std::string GetStrFromType(const RobotBase::RobotType& type){
        for(const auto& kv : RobotBase::RobotTypeMap){
            if(kv.second == type){
                return kv.first;
            }
        }
        throw std::invalid_argument("[RobotBase::RobotType::GetStrFromType] Invalid type");
    }

}
