#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <sstream>
#include <random>


#include "lidar_config.cpp"
#include "fake_lidar_file_reader.cpp"

#include "sensor_msgs/msg/laser_scan.hpp"

class FakeLidar: public rclcpp::Node
{
    public:
        FakeLidar();

    private:
        LidarConfig _config;
        std::unique_ptr<FakeLidarFileReader> _fakeLidarFileReader;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scan_publisher;
        rclcpp::TimerBase::SharedPtr _scan_timer;

    private:
        void _publish_fake_scan();
        bool _ranges_are_in_range(const std::vector<float>& ranges, float min, float max);
        std::vector<float> _add_gausian_noise(const std::vector<float>& ranges, float standard_deviation);
};

FakeLidar::FakeLidar() : Node("fake_lidar")
{
    
    _config.declare_parameters(this);
    _config.update_parameters(this);
    _config.print_config(this);

    //_fakeLidarFileReader = std::make_unique<FakeLidarFileReader>("/home/bartek/Desktop/ROS2/tutorial_workspace/src/fake_sensors/data/fake_lidar_readings.csv", this);
    _fakeLidarFileReader = std::make_unique<FakeLidarFileReader>(_config.fake_data_file_path, this);

    _scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("scan",10);

    _scan_timer = this->create_wall_timer(
        std::chrono::milliseconds(_config.get_scan_period_ms()),
        std::bind(&FakeLidar::_publish_fake_scan, this)
    );
}
    
void FakeLidar::_publish_fake_scan()
{
    auto msg = sensor_msgs::msg::LaserScan();
    
    msg.header.stamp = this->now();
    msg.header.frame_id = "lidar_laser_frame";

    msg.angle_min = _config.angle.first;
    msg.angle_max = _config.angle.second;
    msg.range_min = _config.range.first;
    msg.range_max = _config.range.second;
    msg.time_increment = 0;
    msg.angle_increment = _config.get_scan_step();
    msg.scan_time = _config.get_scan_period_ms()/1000.0;

    auto ranges = _fakeLidarFileReader->get_next_fake_reading();

    if(!_ranges_are_in_range(ranges, _config.range.first, _config.range.second))
    {
        RCLCPP_ERROR(this->get_logger(), "Fake lidar ranges out of range");
        return;
    }

    msg.ranges = _add_gausian_noise(ranges, _config.noise_level);

    _scan_publisher->publish(msg);
}

bool FakeLidar::_ranges_are_in_range(const std::vector<float>& ranges, float min, float max)
{
    for (const auto& range : ranges) {
        if (range < min || range > max) {
            return false;
        }
    }

    return true;
}

std::vector<float> FakeLidar::_add_gausian_noise(const std::vector<float>& ranges, float standard_deviation)
{
    std::vector<float> noisy_ranges = ranges;

    if(standard_deviation == 0.0f)
        return noisy_ranges;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> dist(0.0, standard_deviation);

    for (auto& val : noisy_ranges) {
        val += dist(gen);
    }

    return noisy_ranges;
}



int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FakeLidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
 