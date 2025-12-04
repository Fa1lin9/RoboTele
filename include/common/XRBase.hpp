#pragma once
#include <iostream>
#include <unordered_map>

namespace XRBase {

    enum XRType{
        VisionPro
    };

    const std::unordered_map<std::string, XRBase::XRType> XRTypeMap = {
        {"VisionPro", XRBase::XRType::VisionPro}
    };

    static XRBase::XRType GetTypeFromStr(const std::string& str){
        auto temp = XRBase::XRTypeMap.find(str);
        if(temp != XRBase::XRTypeMap.end()){
            return temp->second;
        }

        throw std::invalid_argument("[XRBase::XRType::GetTypeFromStr] Invalid string");
    }

    static std::string GetStrFromType(const XRBase::XRType& type){
        for(const auto& kv : XRBase::XRTypeMap){
            if(kv.second == type){
                return kv.first;
            }
        }
        throw std::invalid_argument("[XRBase::XRType::GetStrFromType] Invalid type");
    }
}
