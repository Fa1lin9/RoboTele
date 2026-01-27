#include <Ros2Bridge/Ros2Bridge.h>

using namespace eprosima::fastdds::dds;

static std::unordered_map<Ros2Bridge::MsgType, std::function<TypeSupport()>> typeFactory = {
       { Ros2Bridge::MsgType::JointState, []() {
            return TypeSupport(new humanoid_msgs::msg::JointStatePubSubType());
       }},
       { Ros2Bridge::MsgType::Demo, []() {
            return TypeSupport(new my_msgs::msg::DemoPubSubType());
       }},
       { Ros2Bridge::MsgType::JointStateWithoutStamp, []() {
            return TypeSupport(new ti5_interfaces::msg::JointStateWithoutStampPubSubType());
       }},
   };

static TypeSupport CreateType(const Ros2Bridge::MsgType& type){
    auto it = typeFactory.find(type);
    if(it != typeFactory.end() ){
        return it->second();
    }else{
        return TypeSupport();
    }
}

Ros2Bridge::Ros2Bridge(){

}

Ros2Bridge::Ros2Bridge(const Ros2Bridge::BasicConfig& config)
{
    LOG_FUNCTION;

    this->Init(config);
}

bool Ros2Bridge::Init(const Ros2Bridge::BasicConfig &config){
    LOG_FUNCTION;
    topicName = config.topicName;
    msgType = config.msgType;

    // Initialize the basic parameter
    if(topicName.empty()){
        throw(std::logic_error("[Ros2Bridge] The name of the topic can not be empty!"));
        return false;
    }
    if(topicName[0] == '/'){
        topicName.insert(0, "rt");
    }else{
        topicName.insert(0, "rt/");
    }

    // Step 1 创建 DomainParticipant
    participant = DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
    if (!participant)
    {
        throw(std::logic_error("[Ros2Bridge] Failed to create DomainParticipant!"));
        return false;
    }

    // Step 2 注册类型
    type = CreateType(config.msgType);
    if (type.register_type(participant) != RETCODE_OK)
    {
        throw(std::logic_error("[Ros2Bridge] Failed to register type!"));
        return false;
    }

    // Step 3 创建 Topic

    topic = participant->create_topic(topicName, type.get_type_name(), TOPIC_QOS_DEFAULT);
    if (!topic)
    {
        throw(std::logic_error("[Ros2Bridge] Failed to create topic!"));
        return false;
    }

    // Step 4 创建 Publisher
    publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    if (!publisher)
    {
        throw(std::logic_error("[Ros2Bridge] Failed to create publisher!"));
        return false;
    }

    // Step 5 创建 DataWriter
    DataWriterQos writerQos = DATAWRITER_QOS_DEFAULT;
    writer = publisher->create_datawriter(topic, writerQos, nullptr);
    if (!writer)
    {
        throw(std::logic_error("[Ros2Bridge] Failed to create datawriter!"));
        return false;
    }

    std::cout << "[Ros2Bridge] Ros2Bridge initialized successfully." << std::endl;
    return true;
}

Ros2Bridge::~Ros2Bridge(){
    if (participant)
    {
        participant->delete_contained_entities();
        DomainParticipantFactory::get_instance()->delete_participant(participant);
        std::cout << "[Ros2Bridge] Cleaned up FastDDS entities." << std::endl;
    }
}

const std::unordered_map<std::string, Ros2Bridge::MsgType> Ros2Bridge::typeMap = {
    {"JointStateWithoutStamp", Ros2Bridge::MsgType::JointStateWithoutStamp},
    {"JointState", Ros2Bridge::MsgType::JointState},
    {"Demo", Ros2Bridge::MsgType::Demo},
};

Ros2Bridge::MsgType Ros2Bridge::GetMsgTypeFromStr(const std::string& str){
    auto temp = Ros2Bridge::typeMap.find(str);
    if(temp != Ros2Bridge::typeMap.end()){
        return temp->second;
    }

    throw std::invalid_argument("[Ros2Bridge::GetTypeFromStr] Invalid string");
}


