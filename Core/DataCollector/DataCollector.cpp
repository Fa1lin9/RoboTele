#include <DataCollector/DataCollector.hpp>

#include "VisionProData.pb.h"

RobotTeleoperate::HandPositions temp;
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

    leftHandPositions.setZero();
    rightHandPositions.setZero();

    std::cout << FUNC_SIG <<" initialized, connected to " << address << std::endl;
}

DataCollector::~DataCollector(){

}

void DataCollector::run(){
    LOG_FUNCTION;

    while (true) {
        zmq::message_t msg;
        subscriber.recv(msg, zmq::recv_flags::none);

//        if (msg.size() != sizeof(Eigen::Matrix4d) * 3) {
//            std::cout<<"The length should be :"<<sizeof(Eigen::Matrix4d) * 3<<std::endl;
//            std::cerr << "Received wrong size: " << msg.size() << std::endl;
//            continue;
//        }


        std::lock_guard<std::mutex> lock(mutex);

        // Get Data
//            std::memcpy(this->headPose.data(), msg.data(), sizeof(Eigen::Matrix4d));
//            std::memcpy(this->leftArmPose.data(), (char*)msg.data() + sizeof(Eigen::Matrix4d), sizeof(Eigen::Matrix4d));
//            std::memcpy(this->rightArmPose.data(), (char*)msg.data() + sizeof(Eigen::Matrix4d) * 2, sizeof(Eigen::Matrix4d));

        RobotTeleoperate::VisionProData data;

        if (!data.ParseFromArray(msg.data(), msg.size())) {
            throw std::logic_error("Failed to parse protobuf data!");
        }

        // === 1. 读取 headPose ===
        if (data.headPose().data_size() == 16) {
            for (int i = 0; i < 16; i++)
                this->headPose(i / 4, i % 4) = data.headPose().data(i);
        }

        // === 2. 读取 leftArmPose ===
        if (data.leftArmPose().data_size() == 16) {
            for (int i = 0; i < 16; i++)
                this->leftArmPose(i / 4, i % 4) = data.leftArmPose().data(i);
        }

        // === 3. 读取 rightArmPose ===
        if (data.rightArmPose().data_size() == 16) {
            for (int i = 0; i < 16; i++)
                this->rightArmPose(i / 4, i % 4) = data.rightArmPose().data(i);
        }

        // Transpose
//            this->headPose.transposeInPlace();
//            this->leftArmPose.transposeInPlace();
//            this->rightArmPose.transposeInPlace();


        // Matrix Output
        if(0){
            std::cout << "--------------------------" << std::endl;
            std::cout << "Head Pose:\n" << headPose << std::endl;
            std::cout << "Left Wrist Pose:\n" << leftArmPose << std::endl;
            std::cout << "Right Wrist Pose:\n" << rightArmPose << std::endl;
            std::cout << "--------------------------" << std::endl;
        }
    }

}

std::vector<Eigen::Matrix4d> DataCollector::GetValue(){
//    LOG_FUNCTION;
    zmq::message_t msg;
//    subscriber.set(zmq::sockopt::conflate, 1);47


    subscriber.recv(msg, zmq::recv_flags::none);

//    if (msg.size() != sizeof(Eigen::Matrix4d) * 3) {
//        std::cout<<"The length should be :"<<sizeof(Eigen::Matrix4d) * 3<<std::endl;
//        std::cerr << "Received wrong size: " << msg.size() << std::endl;
//        return {};
//    }

//    // Get Data
//    std::memcpy(this->headPose.data(), msg.data(), sizeof(Eigen::Matrix4d));
//    std::memcpy(this->leftArmPose.data(), (char*)msg.data() + sizeof(Eigen::Matrix4d), sizeof(Eigen::Matrix4d));
//    std::memcpy(this->rightArmPose.data(), (char*)msg.data() + sizeof(Eigen::Matrix4d) * 2, sizeof(Eigen::Matrix4d));

//    // Transpose
//    this->headPose.transposeInPlace();
//    this->leftArmPose.transposeInPlace();
//    this->rightArmPose.transposeInPlace();

    RobotTeleoperate::VisionProData data;

    if (!data.ParseFromArray(msg.data(), msg.size())) {
        throw std::logic_error("Failed to parse protobuf data!");
    }

    // === 1. 读取 headPose ===
    if (data.headPose().data_size() == 16) {
        for (int i = 0; i < 16; i++)
            this->headPose(i / 4, i % 4) = data.headPose().data(i);
    }

    // === 2. 读取 leftArmPose ===
    if (data.leftArmPose().data_size() == 16) {
        for (int i = 0; i < 16; i++)
            this->leftArmPose(i / 4, i % 4) = data.leftArmPose().data(i);
    }

    // === 3. 读取 rightArmPose ===
    if (data.rightArmPose().data_size() == 16) {
        for (int i = 0; i < 16; i++)
            this->rightArmPose(i / 4, i % 4) = data.rightArmPose().data(i);
    }

    // Matrix Output
    if(0){
        std::cout << "--------------------------" << std::endl;
        std::cout << "Head Pose:\n" << headPose << std::endl;
        std::cout << "Left Wrist Pose:\n" << leftArmPose << std::endl;
        std::cout << "Right Wrist Pose:\n" << rightArmPose << std::endl;
        std::cout << "--------------------------" << std::endl;
    }

//    std::lock_guard<std::mutex> lock(mutex);
    return {this->headPose, this->leftArmPose, this->rightArmPose};
}
