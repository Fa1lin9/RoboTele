#pragma once
#include <iostream>
#include <array>
#include <unordered_map>

namespace XRBase {

    enum XRType{
        VisionPro
    };

    namespace VisionProConfig {
        // Actually 27: 25 + 1(repeated joint) + 1(world)
        constexpr int NumJoints = 25;

        inline constexpr std::array<size_t, 4> ThumbFingerJointsIndex = {1, 2, 3, 4};
        inline constexpr std::array<size_t, 5> IndexFingerJointsIndex = {5 ,6, 7, 8, 9};
        inline constexpr std::array<size_t, 5> MiddleFingerJointsIndex = {10, 11, 12, 13, 14};
        inline constexpr std::array<size_t, 5> RingFingerJointsIndex = {15, 16, 17, 18, 19};
        inline constexpr std::array<size_t, 5> LittleFingerJointsIndex = {20, 21, 22, 23, 24};
    }

    const std::unordered_map<std::string, XRBase::XRType> XRTypeMap = {
        {"VisionPro", XRBase::XRType::VisionPro}
    };

    inline XRBase::XRType GetTypeFromStr(const std::string& str){
        auto temp = XRBase::XRTypeMap.find(str);
        if(temp != XRBase::XRTypeMap.end()){
            return temp->second;
        }

        throw std::invalid_argument("[XRBase::XRType::GetTypeFromStr] Invalid string");
    }

    inline std::string GetStrFromType(const XRBase::XRType& type){
        for(const auto& kv : XRBase::XRTypeMap){
            if(kv.second == type){
                return kv.first;
            }
        }
        throw std::invalid_argument("[XRBase::XRType::GetStrFromType] Invalid type");
    }
}
