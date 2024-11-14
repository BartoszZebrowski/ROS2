#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <sstream>
#include <random>


#include "lidar_config.cpp"
#include "fake_lidar_file_reader.cpp"

#include "sensor_msgs/msg/laser_scan.hpp"


// Fałszywy lidar powinien wczytywać dane odczytów z pliku CSV, ilość odczytów na pełny skan powinna odpowiadać
// ilości kolumn w pliku CSV. Pozostałe elementy konfiguracji lidaru powinny być ustawiane jako parametry ROS2.
// Jeżeli w pliku CSV znajdą się odczyty przekraczające zakres ustalony w parametrach odczyt taki nie powinien zostać opublikowany
// a węzeł powinien zalogować o tym informacje na poziomie ERROR.
// Dodatkowo należy umożliwić dodanie szumu Gaussowskiego gdzie wartość oczekiwana to zawsze zero a odchylenie standardowe podawane jest jako parametr ROS2. W przypadku odchylenia zero nie należy wcale dodawać szumu.



class FakeLidar: public rclcpp::Node
{
    public:
        FakeLidar();

    private:
        LidarConfig _config;
        std::unique_ptr<FakeLidarFileReader> _fakeLidarFileReader;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scan_publisher;
        rclcpp::TimerBase::SharedPtr _scan_timer;

        // std::default_random_engine generator;
        // std::normal_distribution<double> dist(mean, stddev);

    private:
        void _publish_fake_scan();
        bool _ranges_are_in_range(const std::vector<float>& ranges, float min, float max);

};

FakeLidar::FakeLidar() : Node("fake_lidar")
{
    _fakeLidarFileReader = std::make_unique<FakeLidarFileReader>("/home/bartek/Desktop/ROS2/tutorial_workspace/src/fake_sensors/data/fake_lidar_readings.csv", this);

    _config.declare_parameters(this);
    _config.update_parameters(this);
    _config.print_config(this);

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
    msg.angle_increment = _config.get_scan_step();
    msg.time_increment = 0;
    msg.scan_time = _config.get_scan_period_ms()/1000.0;

    auto ranges = _fakeLidarFileReader->get_next_fake_reading();

    if(!_ranges_are_in_range(ranges, _config.range.first, _config.range.second))
    {
        RCLCPP_ERROR(this->get_logger(), "Fake lidar ranges out of range");
        return;
    }



    msg.ranges = ranges;

    _scan_publisher->publish(msg);
}

bool FakeLidar::_ranges_are_in_range(const std::vector<float>& ranges, float min, float max){
    return std::any_of(ranges.begin(), ranges.end(), [min, max](float val) {
        return val < min && val > max;
    });
}

//=================================================================================================
//                                          MAIN 
//=================================================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FakeLidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
 