#pragma once
#ifdef ON
#undef ON
#endif
#ifdef OFF
#undef OFF
#endif
#ifdef AUTO
#undef AUTO
#endif
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#include <FunctionLogger.hpp>

using namespace eprosima::fastdds::dds;

// msgs's header
// JointState
#include <JointState/JointState.hpp>
#include <JointState/JointStatePubSubTypes.hpp>
// Demo
#include <Demo/Demo.hpp>
#include <Demo/DemoPubSubTypes.hpp>
// JointStateWithoutStamp
#include <JointStateWithoutStamp/JointStateWithoutStamp.hpp>
#include <JointStateWithoutStamp/JointStateWithoutStampPubSubTypes.hpp>

#include <unordered_map>

class Ros2Bridge
{
public:
    enum MsgType{
        JointStateWithoutStamp,
        JointState,
        Demo,
    };

    static std::unordered_map<Ros2Bridge::MsgType, std::function<TypeSupport()>> typeFactory;


    struct BasicConfig
    {
        std::string topicName;
        Ros2Bridge::MsgType msgType;
    };

    Ros2Bridge();
    Ros2Bridge(const Ros2Bridge::BasicConfig& config);
    ~Ros2Bridge();

    bool Init(const Ros2Bridge::BasicConfig& config);

    template<typename T>
    bool SendMsg(const T& msg);

    static Ros2Bridge::MsgType GetMsgTypeFromStr(const std::string& str);

private:
    static const std::unordered_map<std::string, Ros2Bridge::MsgType> typeMap;

    std::string topicName;
    MsgType msgType;

    DomainParticipant* participant;
    Publisher* publisher;
    Topic* topic;
    DataWriter* writer;
    TypeSupport type;


};

static TypeSupport CreateType(const Ros2Bridge::MsgType& type);

template <typename T>
bool Ros2Bridge::SendMsg(const T& msg){
    if(!writer){
        throw(std::logic_error("[Ros2Bridge] Writer not initialized!"));
    }

    T msgCopy = msg;
    ReturnCode_t ret = writer->write(&msgCopy);
    if(ret != RETCODE_OK){
        std::cerr<<"[Ros2Bridge] Failed to send messages!"<<std::endl;
        return false;
    }else{
        std::cout<<"[Publisher] Sent "<<type.get_type_name()<<std::endl;
    }
    return true;
}
