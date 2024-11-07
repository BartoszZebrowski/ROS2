#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <sstream>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>



class MinimalPublisher : public rclcpp::Node
{
    public: 
        MinimalPublisher(const char* node_name);

    private:
        rclcpp::TimerBase::SharedPtr _bragging_coutner_timer;
        rclcpp::TimerBase::SharedPtr _color_timer;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _bragging_coutner_publisher;
        rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr _color_publisher;

        int _count;

    private:
        void bragging_counter_callback();
        void color_callback();
};

MinimalPublisher::MinimalPublisher(const char* node_name) : Node(node_name)
{
    this->declare_parameter("count_start", 0);
    _count = this->get_parameter("count_start").as_int();

    _bragging_coutner_publisher = this->create_publisher<std_msgs::msg::String>("bragging",10);
    _color_publisher = this->create_publisher<std_msgs::msg::ColorRGBA>("color",10);

    _bragging_coutner_timer = this->create_wall_timer(std::chrono::milliseconds(1000),
                                                     std::bind(&MinimalPublisher::bragging_counter_callback, this)); 
                                                     
    _color_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                     std::bind(&MinimalPublisher::color_callback, this)); 

}

void MinimalPublisher::bragging_counter_callback()
{
    auto message = std_msgs::msg::String();
    std::stringstream str_stream;
    str_stream << "I have already counted " << _count;
    _count++;
    message.set__data(str_stream.str());

    RCLCPP_INFO(this->get_logger(), "PUBLISHING : %s", message.data.c_str());

    _bragging_coutner_publisher->publish(message);
}

void MinimalPublisher::color_callback()
{
    auto messageColor = std_msgs::msg::ColorRGBA();

    messageColor.r = 3.14;
    messageColor.g = 1.14;
    messageColor.b = 4.14;
    messageColor.a = 1;

    RCLCPP_INFO(this->get_logger(), "PUBLISHING : [%f,%f,%f,%f,]]", messageColor.r, messageColor.g, messageColor.b, messageColor.a);

    _color_publisher->publish(messageColor);
}










// =======================================

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    
    auto node = std::make_shared<MinimalPublisher>("minimal_publisher");

    rclcpp::spin(node);
    rclcpp::shutdown();


    return 0;
}
 