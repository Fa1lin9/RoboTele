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
        ROHand,
        Revo2Hand,
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
            36, 102, 98, 102, 100, 2
        };

        inline constexpr std::array<double, NumFingers> FingersUpperBound = {
            2, 178, 176, 176, 174, 88
        };

        inline std::array<const char*, NumFingers> FingersName = {
            "if_slider_link",
            "mf_slider_link",
            "rf_slider_link",
            "lf_slider_link",
            "th_slider_link",
            "th_root_link"
        };
    }

    namespace Revo2HandConfig {
        // Doucment
        // https://github.com/BrainCoTech/revo2_description/blob/main/README_CN.md

        /*
        =========================== 左手关节 (Left Hand) ===========================

        | 关节名称                  | 描述       | 角度范围(度) | 角度范围(弧度) |
        |---------------------------|------------|--------------|----------------|
        | left_thumb_flex_joint     | 拇指屈伸   | 0 ~ 59       | 0 ~ 1.03       |
        | left_thumb_abduct_joint   | 拇指外展   | 0 ~ 90       | 0 ~ 1.57       |
        | left_index_joint          | 食指       | 0 ~ 81       | 0 ~ 1.41       |
        | left_middle_joint         | 中指       | 0 ~ 81       | 0 ~ 1.41       |
        | left_ring_joint           | 无名指     | 0 ~ 81       | 0 ~ 1.41       |
        | left_pinky_joint          | 小指       | 0 ~ 81       | 0 ~ 1.41       |


        =========================== 右手关节 (Right Hand) ===========================

        | 关节名称                  | 描述       | 角度范围(度) | 角度范围(弧度) |
        |---------------------------|------------|--------------|----------------|
        | right_thumb_flex_joint    | 拇指屈伸   | 0 ~ 59       | 0 ~ 1.03       |
        | right_thumb_abduct_joint  | 拇指外展   | 0 ~ 90       | 0 ~ 1.57       |
        | right_index_joint         | 食指       | 0 ~ 81       | 0 ~ 1.41       |
        | right_middle_joint        | 中指       | 0 ~ 81       | 0 ~ 1.41       |
        | right_ring_joint          | 无名指     | 0 ~ 81       | 0 ~ 1.41       |
        | right_pinky_joint         | 小指       | 0 ~ 81       | 0 ~ 1.41       |

        */

        inline constexpr int NumFingers = 6;

//        inline constexpr std::array<double, NumFingers> FingersLowerBound = {
//            2, 2, 2, 2, 2, 2
//        };

//        inline constexpr std::array<double, NumFingers> FingersUpperBound = {
//            58, 80, 80, 80, 80, 88
//        };

        inline constexpr std::array<double, NumFingers> FingersLowerBound = {
            2, 80, 80, 80, 80, 2
        };

        inline constexpr std::array<double, NumFingers> FingersUpperBound = {
            58, 2, 2, 2, 2, 88
        };

        inline std::array<const char*, 11> LeftFingersName = {
            "left_thumb_metacarpal_joint",
            "left_thumb_proximal_joint",
            "left_thumb_distal_joint",
            "left_index_proximal_joint",
            "left_index_distal_joint",
            "left_middle_proximal_joint",
            "left_middle_distal_joint",
            "left_ring_proximal_joint",
            "left_ring_distal_joint",
            "left_pinky_proximal_joint",
            "left_pinky_distal_joint"
        };

        inline std::array<const char*, 11> RightFingersName = {
            "right_thumb_metacarpal_joint",
            "right_thumb_proximal_joint",
            "right_thumb_distal_joint",
            "right_index_proximal_joint",
            "right_index_distal_joint",
            "right_middle_proximal_joint",
            "right_middle_distal_joint",
            "right_ring_proximal_joint",
            "right_ring_distal_joint",
            "right_pinky_proximal_joint",
            "right_pinky_distal_joint"
        };
    }

    inline const std::unordered_map<std::string, HandBase::HandType> HandTypeMap = {
        {"ROHand", HandBase::HandType::ROHand},
        {"Revo2Hand", HandBase::HandType::Revo2Hand},
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
