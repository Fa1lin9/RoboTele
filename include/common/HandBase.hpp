#pragma once
#include <iostream>
#include <unordered_map>
#include <Eigen/Dense>
#include <array>

namespace HandBase{
    struct HandGesture{
        // For VisionPro
        bool pinchState;
        double pinchValue;
        bool squeezeState;
        double squeezeValue;
    };

    enum HandType{
        ROHand
    };

    struct HandData{
        std::vector<Eigen::Vector3d> handPositions;

        HandGesture handGesture;

    };

    struct DualHandData{
        HandData leftHandData;

        HandData rightHandData;

        HandBase::HandType type;
    };

    namespace ROHandConfig {
        // https://github.com/oymotion/roh_gen2_firmware/blob/main/protocol/OHandModBusRTUProtocol_CN.md

        // ID       Joint   Lower(deg)  Upper(deg)
        // thumb    0       2.26        36.76
        // index    1       100.22      178.37
        // middle   2       97.81       176.06
        // ring     3       101.38      176.54
        // little   4       98.84       174.86
        // thumbRot 5       0           90
            inline constexpr int NumFingers = 6;

            inline constexpr std::array<double, NumFingers> FingersLowerBound = {
                2, 102, 98, 102, 100, 2
            };

            inline constexpr std::array<double, NumFingers> FingersUpperBound = {
                36, 178, 176, 176, 174, 88
            };

            inline std::array<const char*, NumFingers> FingersName = {
                "thumb",
                "index",
                "middle",
                "ring",
                "little",
                "thumbRot"
            };
        };

    inline const std::unordered_map<std::string, HandBase::HandType> HandTypeMap = {
        {"ROHand", HandBase::HandType::ROHand}
    };

    inline HandBase::HandType GetTypeFromStr(const std::string& str){
        auto temp = HandBase::HandTypeMap.find(str);
        if(temp != HandBase::HandTypeMap.end()){
            return temp->second;
        }

        throw std::invalid_argument("[HandBase::HandType::GetTypeFromStr] Invalid string");
    }

    inline std::string GetStrFromType(const HandBase::HandType& type){
        for(const auto& kv : HandBase::HandTypeMap){
            if(kv.second == type){
                return kv.first;
            }
        }
        throw std::invalid_argument("[HandBase::HandType::GetStrFromType] Invalid type");
    }

}
