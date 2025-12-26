#include <VisionPro2Ti5RobotTransform/VisionPro2Ti5RobotTransform.hpp>
VisionPro2Ti5RobotTransform::VisionPro2Ti5RobotTransform(const Transform::BasicConfig &config_)
    :T_Head2Waist(config_.T_Head2Waist),
     T_XR2Robot(config_.T_XR2Robot),
     T_Robot2LeftWrist(config_.T_Robot2LeftWrist),
     T_Robot2RightWrist(config_.T_Robot2RightWrist),
     offset(config_.offset)
{

}

VisionPro2Ti5RobotTransform::~VisionPro2Ti5RobotTransform(){

}

std::vector<Eigen::Matrix4d> VisionPro2Ti5RobotTransform::Solve(
        const Transform::MsgConfig &config_){

    // From XR to Robot
    Eigen::Matrix4d head2RobotWorldPose =
            T_XR2Robot * config_.head2XRWorldPose * T_XR2Robot.inverse();
    Eigen::Matrix4d leftWrist2RobotWorldPose =
            T_XR2Robot * config_.leftWrist2XRWorldPose * T_XR2Robot.inverse();
    Eigen::Matrix4d rightWrist2RobotWorldPose =
            T_XR2Robot * config_.rightWrist2XRWorldPose * T_XR2Robot.inverse();
    Eigen::Matrix4d initHead2RobotWorldPose =
            T_XR2Robot * config_.initHeadPose * T_XR2Robot.inverse();
    // Coincide with the wrist coordinate system defined in URDF
    leftWrist2RobotWorldPose = leftWrist2RobotWorldPose * T_Robot2LeftWrist;
    rightWrist2RobotWorldPose = rightWrist2RobotWorldPose * T_Robot2RightWrist;

    // Mode Selection
    Eigen::Matrix4d upperBody2RobotWorldPose;
    if (config_.mode == Transform::TeleMode::HeadMode){
        if (config_.modeHeadPose.has_value()){
            upperBody2RobotWorldPose = config_.modeHeadPose.value();
            upperBody2RobotWorldPose =
                    T_XR2Robot * upperBody2RobotWorldPose * T_XR2Robot.inverse();
//            std::cout << "HeadPose: " << head2RobotWorldPoseLocked << std::endl;

            // Rotation
            Eigen::Vector3d rpy =
                    MatrixUtils::RotationToEulerZYX(upperBody2RobotWorldPose.block<3,3>(0,0));

//            std::cout << "HeadPose.yaw: " << rpy(2) << std::endl;
            Eigen::Matrix3d rot = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()).toRotationMatrix();

            upperBody2RobotWorldPose.block<3,3>(0,0) = rot;

//            std::cout << "HeadPose: " << head2RobotWorldPoseLocked << std::endl;

        } else{
            // previous version
//            upperBody2RobotWorldPose = Eigen::Matrix4d::Identity();
//            // X
//            upperBody2RobotWorldPose(0,3) = 0;
//            // Y
//            upperBody2RobotWorldPose(1,3) = 0;
//            // Z
//            upperBody2RobotWorldPose(2,3) = 1.1;
//            // Rotation
//            upperBody2RobotWorldPose.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

            // Current Version
            Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
            temp.block<3,1>(0,3) = initHead2RobotWorldPose.block<3,1>(0,3);
            upperBody2RobotWorldPose = temp;
        }

        // To Head
        // only for translation
//        leftWrist2RobotWorldPose.block<3,1>(0,3) =
//                leftWrist2RobotWorldPose.block<3,1>(0,3) - upperBody2RobotWorldPose.block<3,1>(0,3);
//        rightWrist2RobotWorldPose.block<3,1>(0,3) =
//                rightWrist2RobotWorldPose.block<3,1>(0,3) - upperBody2RobotWorldPose.block<3,1>(0,3);
            leftWrist2RobotWorldPose.block<3,1>(0,3) =
                    (upperBody2RobotWorldPose.inverse() * leftWrist2RobotWorldPose).block<3,1>(0,3);
            rightWrist2RobotWorldPose.block<3,1>(0,3) =
                    (upperBody2RobotWorldPose.inverse() * rightWrist2RobotWorldPose).block<3,1>(0,3);
    }else if (config_.mode == Transform::TeleMode::WaistMode)
    {
        // To Head
        Eigen::Matrix4d head2RobotWorldPoseInv = head2RobotWorldPose.inverse();
        leftWrist2RobotWorldPose = head2RobotWorldPoseInv * leftWrist2RobotWorldPose;
        rightWrist2RobotWorldPose = head2RobotWorldPoseInv * rightWrist2RobotWorldPose;
//        leftWrist2RobotWorldPose.block<3,1>(0,3) =
//                (head2RobotWorldPoseInv * leftWrist2RobotWorldPose).block<3,1>(0,3);
//        rightWrist2RobotWorldPose.block<3,1>(0,3) =
//                (head2RobotWorldPoseInv * rightWrist2RobotWorldPose).block<3,1>(0,3);
    }

    // Offset
    leftWrist2RobotWorldPose.block<3,1>(0,3) += offset;
    rightWrist2RobotWorldPose.block<3,1>(0,3) += offset;

    // To Waist
    Eigen::Matrix4d LeftWrist2RobotWaistPose = T_Head2Waist * leftWrist2RobotWorldPose;
    Eigen::Matrix4d RightWrist2RobotWaistPose = T_Head2Waist * rightWrist2RobotWorldPose;

    // Actually to Head
    return {LeftWrist2RobotWaistPose , RightWrist2RobotWaistPose, head2RobotWorldPose, upperBody2RobotWorldPose};
}
