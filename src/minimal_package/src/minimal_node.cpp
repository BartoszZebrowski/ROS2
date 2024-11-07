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
    _timer = this->create_wall_timer(std::chrono::milliseconds(1000),
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
    auto node = MinimalNode::make_shared("minimal_node");

    rclcpp::spin(node);


    return 0;
}
