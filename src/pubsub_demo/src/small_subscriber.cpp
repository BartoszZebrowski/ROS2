#include "rclcpp/rclcpp.hpp"
#include <memory>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>

typedef std_msgs::msg::String StringMsg;

class MinimalSubscriber : public rclcpp::Node
{
    public: 
        MinimalSubscriber(const char* node_name);

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _bragging_counter_subscruption;

    private:
        void bragging_counter_callback(StringMsg::UniquePtr msg);
};

MinimalSubscriber::MinimalSubscriber(const char* node_name) : Node(node_name) 
{
    _bragging_counter_subscruption = this->create_subscription<StringMsg>("bragging", 10, std::bind(&MinimalSubscriber::bragging_counter_callback, this, std::placeholders::_1));
}

void MinimalSubscriber::bragging_counter_callback(StringMsg::UniquePtr message){
    RCLCPP_INFO(this->get_logger(), "RECEIVED - %s", message->data.c_str());
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    
    auto node = std::make_shared<MinimalSubscriber>("minimal_subscriber");

    rclcpp::spin(node);
    rclcpp::shutdown();


    return 0;
}