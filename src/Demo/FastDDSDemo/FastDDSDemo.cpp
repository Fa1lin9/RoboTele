#include <Ros2Bridge/Ros2Bridge.h>

using namespace eprosima::fastdds::dds;

int main()
{
    Ros2Bridge::BasicConfig config;
    config.msgType = Ros2Bridge::MsgType::Demo;
    config.topicName = "StringTopic";
    Ros2Bridge ros2Bridge(config);

    my_msgs::msg::Demo msg;
    int count = 0;

    while (true)
    {
        msg.name() = "Hello FastDDS! Count = " + std::to_string(count++);
        ros2Bridge.SendMsg(msg);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
