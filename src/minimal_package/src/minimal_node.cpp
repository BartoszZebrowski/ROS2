#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>


class MinimalNode : public rclcpp::Node
{
    public:
        MinimalNode(const char* node_name);

    private:
        rclcpp::TimerBase::SharedPtr _timer;

    private:
        void _timer_callback(); 
};

MinimalNode::MinimalNode(const char* node_name) : Node(node_name)
{
    this->declare_parameter("period", 1000);
    int period = this->get_parameter("period").as_int();

    _timer = this->create_wall_timer(std::chrono::milliseconds(period),
                                     std::bind(&MinimalNode::_timer_callback, this)); 
}

void MinimalNode::_timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Test hello world");
}

//=============================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MinimalNode>("minimal_node");

    rclcpp::spin(node);


    return 0;
}
