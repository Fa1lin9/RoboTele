#pragma once
#include <iostream>
#include <unordered_map>
#include <MatrixUtils.hpp>
#include <memory>
#include <shared_mutex>
#include <mutex>

namespace RobotBase{

    struct JointInfo {
        std::string name;
        int index;
        MatrixUtils::EulerAxis type;
        int direction;
    };

    enum RobotType{
        GenericRobot,

        Ti5Robot,
        UnitreeG1Dof29,
        UnitreeG1Dof23,
        CrpRobot,
    };

    inline const std::unordered_map<std::string, RobotBase::RobotType> RobotTypeMap = {
        {"Ti5Robot", RobotBase::RobotType::Ti5Robot},
        {"UnitreeG1Dof29", RobotBase::RobotType::UnitreeG1Dof29},
        {"UnitreeG1Dof23", RobotBase::RobotType::UnitreeG1Dof23},
        {"CrpRobot", RobotBase::RobotType::CrpRobot},

        {"GenericRobot", RobotBase::RobotType::GenericRobot},
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

    namespace UnitreeG1 {

        namespace Dof23 {
            const size_t armDof = 5;
            const size_t headDof = 0;
            const size_t legDof = 6;
            const size_t waistDof = 1;
            const size_t totalDof = 23;
        }

        namespace Dof29 {
            const size_t armDof = 7;
            const size_t headDof = 0;
            const size_t legDof = 6;
            const size_t waistDof = 3;
            // In the Dof29, the totalDof ignore the handDof.
            const size_t totalDof = 29;
            const size_t handDof = 7;
        }

        static const std::string kTopicCmd = "rt/lowcmd";
        static const std::string kTopicState = "rt/lowstate";

        enum JointIndex {
            // Left leg
            kLeftHipPitch = 0,
            kLeftHipRoll = 1,
            kLeftHipYaw = 2,
            kLeftKnee = 3,
            kLeftAnklePitch  = 4,
            kLeftAnkleRoll = 5,

            // Right leg
            kRightHipPitch = 6,
            kRightHipRoll = 7,
            kRightHipYaw = 8,
            kRightKnee = 9,
            kRightAnklePitch = 10,
            kRightAnkleRoll = 11,

            kWaistYaw = 12,
            kWaistRoll = 13,
            kWaistPitch = 14,

            // Left arm
            kLeftShoulderPitch = 15,
            kLeftShoulderRoll = 16,
            kLeftShoulderYaw = 17,
            kLeftElbow = 18,
            kLeftWristRoll = 19,
            kLeftWristPitch = 20,
            kLeftWristYaw = 21,

            // Right arm
            kRightShoulderPitch = 22,
            kRightShoulderRoll = 23,
            kRightShoulderYaw = 24,
            kRightElbow = 25,
            kRightWristRoll = 26,
            kRightWristPitch = 27,
            kRightWristYaw = 28,

            kNotUsedJoint = 29,
            kNotUsedJoint1 = 30,
            kNotUsedJoint2 = 31,
            kNotUsedJoint3 = 32,
            kNotUsedJoint4 = 33,
            kNotUsedJoint5 = 34
        };

        inline std::vector<RobotBase::UnitreeG1::JointIndex> WeakMotor = {
            RobotBase::UnitreeG1::JointIndex::kLeftAnklePitch,
            RobotBase::UnitreeG1::JointIndex::kRightAnklePitch,

            // Left Arm
            RobotBase::UnitreeG1::JointIndex::kLeftShoulderPitch,
            RobotBase::UnitreeG1::JointIndex::kLeftShoulderRoll,
            RobotBase::UnitreeG1::JointIndex::kLeftShoulderYaw,
            RobotBase::UnitreeG1::JointIndex::kLeftElbow,

            // Right Arm
            RobotBase::UnitreeG1::JointIndex::kRightShoulderPitch,
            RobotBase::UnitreeG1::JointIndex::kRightShoulderRoll,
            RobotBase::UnitreeG1::JointIndex::kRightShoulderYaw,
            RobotBase::UnitreeG1::JointIndex::kRightElbow,
        };

        inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
              uint32_t xbit = 0;
              uint32_t data = 0;
              uint32_t CRC32 = 0xFFFFFFFF;
              const uint32_t dwPolynomial = 0x04c11db7;
              for (uint32_t i = 0; i < len; i++) {
                xbit = 1 << 31;
                data = ptr[i];
                for (uint32_t bits = 0; bits < 32; bits++) {
                  if (CRC32 & 0x80000000) {
                    CRC32 <<= 1;
                    CRC32 ^= dwPolynomial;
                  } else
                    CRC32 <<= 1;
                  if (data & xbit) CRC32 ^= dwPolynomial;

                  xbit >>= 1;
                }
              }
              return CRC32;
        };
    }

    // For DataBuffer
    template <typename T>
    class DataBuffer {
     public:
      void SetData(const T &newData) {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data = std::make_shared<T>(newData);
      }

      std::shared_ptr<const T> GetData() {
        std::shared_lock<std::shared_mutex> lock(mutex);
        return data ? data : nullptr;
      }

      void Clear() {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data = nullptr;
      }

     private:
      std::shared_ptr<T> data;
      std::shared_mutex mutex;
    };
}
