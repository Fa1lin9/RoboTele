#pragma once
#include <iostream>
#include <unordered_map>

namespace HandBase{

    enum HandType{
        ROHand
    };

    const std::unordered_map<std::string, HandBase::HandType> HandTypeMap = {
        {"ROHand", HandBase::HandType::ROHand}
    };

    static HandBase::HandType GetTypeFromStr(const std::string& str){
        auto temp = HandBase::HandTypeMap.find(str);
        if(temp != HandBase::HandTypeMap.end()){
            return temp->second;
        }

        throw std::invalid_argument("[HandBase::HandType::GetTypeFromStr] Invalid string");
    }

    static std::string GetStrFromType(const HandBase::HandType& type){
        for(const auto& kv : HandBase::HandTypeMap){
            if(kv.second == type){
                return kv.first;
            }
        }
        throw std::invalid_argument("[HandBase::HandType::GetStrFromType] Invalid type");
    }

}
