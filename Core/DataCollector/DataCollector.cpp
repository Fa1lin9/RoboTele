#include <DataCollector/DataCollector.hpp>

#include "VisionProData.pb.h"

DataCollector::DataCollector(){

}

DataCollector::DataCollector(const std::string &address_)
//     :address(address_),
//      context(1),
//      subscriber(context, zmq::socket_type::sub)
{
    this->Init(address_);

////    LOG_FUNCTION;
//    subscriber.connect(address);
//    subscriber.set(zmq::sockopt::subscribe, "");  // 订阅所有消息

//    // Mat Init
//    headPose.setZero();
//    headPose(3,3) = 1;
//    leftWristPose.setZero();
//    leftWristPose(3,3) = 1;
//    rightWristPose.setZero();
//    rightWristPose(3,3) = 1;

//    leftHandPosition.setZero();
//    rightHandPosition.setZero();

//    std::cout << FUNC_SIG <<" initialized, connected to " << address << std::endl;
}

void DataCollector::Init(const std::string &address_){
    this->address = address_;
    this->context.set(zmq::ctxopt::io_threads, 1);
    this->subscriber = zmq::socket_t(this->context, zmq::socket_type::sub);

//    LOG_FUNCTION;
    subscriber.connect(address);
    subscriber.set(zmq::sockopt::subscribe, "");  // 订阅所有消息

    // Mat Init
    headPose.setZero();
    headPose(3,3) = 1;
    leftArmPose.setZero();
    leftArmPose(3,3) = 1;
    rightArmPose.setZero();
    rightArmPose(3,3) = 1;

//    leftHandPositions.setZero();
//    rightHandPositions.setZero();
    leftHandPositions.resize(this->numJointsHand);
    rightHandPositions.resize(this->numJointsHand);

    std::cout << FUNC_SIG <<" initialized, connected to " << address << std::endl;
}

DataCollector::~DataCollector(){

}

void DataCollector::Run() {
    LOG_FUNCTION;

    while (!this->stopFlag) {
        zmq::message_t msg;

        this->hasNewData = false;

        subscriber.recv(msg, zmq::recv_flags::none);

        {
            std::lock_guard<std::mutex> lock(mutex);

            RobotTeleoperate::VisionProData data;

            if (!data.ParseFromArray(msg.data(), msg.size())) {
                throw std::logic_error("Failed to parse protobuf data!");
            }

            // HeadPose
            if (data.headPose().data_size() == 16) {
                for (int i = 0; i < 16; i++)
                    this->headPose(i / 4, i % 4) = data.headPose().data(i);
            }

            // LeftArmPose
            if (data.leftArmPose().data_size() == 16) {
                for (int i = 0; i < 16; i++)
                    this->leftArmPose(i / 4, i % 4) = data.leftArmPose().data(i);
            }

            // RightArmPose
            if (data.rightArmPose().data_size() == 16) {
                for (int i = 0; i < 16; i++)
                    this->rightArmPose(i / 4, i % 4) = data.rightArmPose().data(i);
            }

            // LeftHandPositions
            {
                int N = data.leftHandPositions().joints_size();
                this->leftHandPositions.resize(N);
                for (int i = 0; i < N; i++) {
                    const auto& p = data.leftHandPositions().joints(i);
                    this->leftHandPositions[i] = Eigen::Vector3d(p.x(), p.y(), p.z());
                }
            }

            // RightHandPositions
            {
                int N = data.rightHandPositions().joints_size();
                this->rightHandPositions.resize(N);
                for (int i = 0; i < N; i++) {
                    const auto& p = data.rightHandPositions().joints(i);
                    this->rightHandPositions[i] = Eigen::Vector3d(p.x(), p.y(), p.z());
                }
            }
        }

        this->hasNewData = true;

        // Debug Output
        if (1) {
            std::cout << "--------------------------" << std::endl;
            std::cout << "Head Pose:\n" << headPose << std::endl;
            std::cout << "Left Wrist Pose:\n" << leftArmPose << std::endl;
            std::cout << "Right Wrist Pose:\n" << rightArmPose << std::endl;

            std::cout << "Left Hand Joints (" << leftHandPositions.size() << "):" << std::endl;
            for (size_t i = 0; i < leftHandPositions.size(); ++i)
                std::cout << i << ": " << leftHandPositions[i].transpose() << std::endl;

            std::cout << "Right Hand Joints (" << rightHandPositions.size() << "):" << std::endl;
            for (size_t i = 0; i < rightHandPositions.size(); ++i)
                std::cout << i << ": " << rightHandPositions[i].transpose() << std::endl;

            std::cout << "--------------------------" << std::endl;
        }


    }
}


//std::vector<Eigen::Matrix4d> DataCollector::GetValue(){
////    LOG_FUNCTION;
//    zmq::message_t msg;

//    subscriber.recv(msg, zmq::recv_flags::none);

//    RobotTeleoperate::VisionProData data;

//    if (!data.ParseFromArray(msg.data(), msg.size())) {
//        throw std::logic_error("Failed to parse protobuf data!");
//    }

//    // === 1. 读取 headPose ===
//    if (data.headPose().data_size() == 16) {
//        for (int i = 0; i < 16; i++)
//            this->headPose(i / 4, i % 4) = data.headPose().data(i);
//    }

//    // === 2. 读取 leftArmPose ===
//    if (data.leftArmPose().data_size() == 16) {
//        for (int i = 0; i < 16; i++)
//            this->leftArmPose(i / 4, i % 4) = data.leftArmPose().data(i);
//    }

//    // === 3. 读取 rightArmPose ===
//    if (data.rightArmPose().data_size() == 16) {
//        for (int i = 0; i < 16; i++)
//            this->rightArmPose(i / 4, i % 4) = data.rightArmPose().data(i);
//    }

//    // Matrix Output
//    if(0){
//        std::cout << "--------------------------" << std::endl;
//        std::cout << "Head Pose:\n" << headPose << std::endl;
//        std::cout << "Left Wrist Pose:\n" << leftArmPose << std::endl;
//        std::cout << "Right Wrist Pose:\n" << rightArmPose << std::endl;
//        std::cout << "--------------------------" << std::endl;
//    }

////    std::lock_guard<std::mutex> lock(mutex);
//    return {this->headPose, this->leftArmPose, this->rightArmPose};
//}

std::vector<Eigen::Matrix4d> DataCollector::GetPoseMatrix(){
    std::lock_guard<std::mutex> lock(this->mutex);
    return {this->headPose, this->leftArmPose, this->rightArmPose};
}

std::vector<Eigen::Vector3d> DataCollector::GetLeftHandPositions(){
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->leftHandPositions;
}

std::vector<Eigen::Vector3d> DataCollector::GetRightHandPositions(){
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->rightHandPositions;
}

bool DataCollector::HasNewData(){
    return this->hasNewData.load();
}

void DataCollector::Stop(){
    this->stopFlag = true;
}
