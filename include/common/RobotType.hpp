#pragma once
#include <iostream>
#include <unordered_map>

namespace RobotType{

    struct JointInfo {
        std::string name;
        int index;
    };

    enum Type{
        Ti5Robot
    };

    const std::unordered_map<std::string, RobotType::Type> RobotTypeMap = {
        {"Ti5Robot", RobotType::Type::Ti5Robot}
    };

    static RobotType::Type GetTypeFromStr(const std::string& str){
        auto temp = RobotType::RobotTypeMap.find(str);
        if(temp != RobotType::RobotTypeMap.end()){
            return temp->second;
        }

        throw std::invalid_argument("[RobotType::GetTypeFromStr] Invalid string");
    }

}
