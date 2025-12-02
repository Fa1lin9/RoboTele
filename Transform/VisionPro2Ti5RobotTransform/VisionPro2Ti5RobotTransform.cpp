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

    // 相似变化到Robot标准坐标系
    Eigen::Matrix4d head2RobotWorldPose =
            T_XR2Robot * config_.head2xrWorldPose * T_XR2Robot.inverse();
    Eigen::Matrix4d leftWrist2RobotWorldPose =
            T_XR2Robot * config_.leftWrist2xrWorldPose * T_XR2Robot.inverse();
    Eigen::Matrix4d rightWrist2RobotWorldPose =
            T_XR2Robot * config_.rightWrist2xrWorldPose * T_XR2Robot.inverse();

    // 与urdf定义的手腕坐标xyz轴重合
    leftWrist2RobotWorldPose = leftWrist2RobotWorldPose * T_Robot2LeftWrist;
    rightWrist2RobotWorldPose = rightWrist2RobotWorldPose * T_Robot2RightWrist;

    // Lock the origin
    if(config_.isLockHead){
        Eigen::Matrix4d head2RobotWorldPoseLocked = Eigen::Matrix4d::Identity();
        // X
        head2RobotWorldPoseLocked(0,3) = 0;
        // Y
        head2RobotWorldPoseLocked(1,3) = 0;
        // Z
//        head2RobotWorldPoseLocked(2,3) =
//                head2RobotWorldPose(2,3);
        head2RobotWorldPoseLocked(2,3) = 1.1;
        // Rotation
        head2RobotWorldPoseLocked.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

        // 转化到头坐标系
    //    head2RobotWorldPose.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
//        Eigen::Matrix4d head2RobotWorldPoseInv = head2RobotWorldPose.inverse();
    //    leftWrist2RobotWorldPose = head2RobotWorldPoseInv * leftWrist2RobotWorldPose;
    //    rightWrist2RobotWorldPose = head2RobotWorldPoseInv * rightWrist2RobotWorldPose;
    //    leftWrist2RobotWorldPose.block<3,1>(0,3) =
    //            (head2RobotWorldPoseInv * leftWrist2RobotWorldPose).block<3,1>(0,3);
    //    rightWrist2RobotWorldPose.block<3,1>(0,3) =
    //            (head2RobotWorldPoseInv * rightWrist2RobotWorldPose).block<3,1>(0,3);

        // only for translation
        leftWrist2RobotWorldPose.block<3,1>(0,3) =
                leftWrist2RobotWorldPose.block<3,1>(0,3) - head2RobotWorldPoseLocked.block<3,1>(0,3);
        rightWrist2RobotWorldPose.block<3,1>(0,3) =
                rightWrist2RobotWorldPose.block<3,1>(0,3) - head2RobotWorldPoseLocked.block<3,1>(0,3);
    }else{
        // 转化到头坐标系
    //    head2RobotWorldPose.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
//        Eigen::Matrix4d head2RobotWorldPoseInv = head2RobotWorldPose.inverse();
    //    leftWrist2RobotWorldPose = head2RobotWorldPoseInv * leftWrist2RobotWorldPose;
    //    rightWrist2RobotWorldPose = head2RobotWorldPoseInv * rightWrist2RobotWorldPose;
    //    leftWrist2RobotWorldPose.block<3,1>(0,3) =
    //            (head2RobotWorldPoseInv * leftWrist2RobotWorldPose).block<3,1>(0,3);
    //    rightWrist2RobotWorldPose.block<3,1>(0,3) =
    //            (head2RobotWorldPoseInv * rightWrist2RobotWorldPose).block<3,1>(0,3);

        // only for translation
        leftWrist2RobotWorldPose.block<3,1>(0,3) =
                leftWrist2RobotWorldPose.block<3,1>(0,3) - head2RobotWorldPose.block<3,1>(0,3);
        rightWrist2RobotWorldPose.block<3,1>(0,3) =
                rightWrist2RobotWorldPose.block<3,1>(0,3) - head2RobotWorldPose.block<3,1>(0,3);

    }

    // 补偿
    leftWrist2RobotWorldPose.block<3,1>(0,3) += offset;
    rightWrist2RobotWorldPose.block<3,1>(0,3) += offset;

    // 转化到腰坐标系
    Eigen::Matrix4d LeftWrist2RobotWaistPose = T_Head2Waist * leftWrist2RobotWorldPose;
    Eigen::Matrix4d RightWrist2RobotWaistPose = T_Head2Waist * rightWrist2RobotWorldPose;

    // 实际上最后就是相对于头的坐标系
    return {LeftWrist2RobotWaistPose , RightWrist2RobotWaistPose, head2RobotWorldPose};
}
