#pragma once

#include <cmath>
#include <utility>

#include "rclcpp/rclcpp.hpp"



class LidarConfig
{
    public:
        const char* PARAM_MIN_RANGE = "min_range";
        const char* PARAM_MAX_RANGE = "max_range";
        const char* PARAM_MIN_ANGLE = "min_angle";
        const char* PARAM_MAX_ANGLE = "max_angle";
        const char* PARAM_SAMPLE_COUNT = "sample_count";
        const char* PARAM_SAMPLING_FREQUENCY = "sampling_frequency";

        const double DEFAULT_MIN_RANGE = 0.2;
        const double DEFAULT_MAX_RANGE = 2.0;
        const double DEFAULT_MIN_ANGLE = -M_PI;
        const double DEFAULT_MAX_ANGLE = M_PI;
        const int DEFAULT_SAMPLE_COUNT = 360;
        const double DEFAULT_SAMPLING_FREQUENCY = 10.0;

        const char* DESCRIPTION_MIN_RANGE =
        "minimum range value [m]";
        const char* DESCRIPTION_MAX_RANGE =
        "maximum range value [m]";
        const char* DESCRIPTION_MIN_ANGLE =
        "start angle of the scan [rad]";
        const char* DESCRIPTION_MAX_ANGLE =
        "end angle of the scan [rad]";
        const char* DESCRIPTION_SAMPLE_COUNT =
        "Number of samples per full laser scan";
        const char* DESCRIPTION_SAMPLING_FREQUENCY =
        "Number of full Scans per second.";

    public:
        std::pair<double,double> range;
        std::pair<double,double> angle;
        int sample_count;
        double sampling_frequency;

    public:
        void declare_parameters(rclcpp::Node *node);
        void update_parameters(rclcpp::Node *node);
        void print_config(rclcpp::Node *node);

        double get_scan_step() const;
        int get_scan_period_ms() const;
        double get_scaled_sample(double scale) const;


};
    
double LidarConfig::get_scaled_sample(double scale) const
{
    return range.first + (range.second - range.first) * scale;
}

int LidarConfig::get_scan_period_ms() const
{
    return std::lround(1000/sampling_frequency);
}

double LidarConfig::get_scan_step() const
{
    return (angle.second-angle.first)/sample_count;
}    

void LidarConfig::declare_parameters(rclcpp::Node *node)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_MIN_RANGE;
    node->declare_parameter(PARAM_MIN_RANGE, DEFAULT_MIN_RANGE, descriptor);

    descriptor.description = DESCRIPTION_MAX_RANGE;
    node->declare_parameter(PARAM_MAX_RANGE, DEFAULT_MAX_RANGE, descriptor);

    descriptor.description = DESCRIPTION_MIN_ANGLE;
    node->declare_parameter(PARAM_MIN_ANGLE, DEFAULT_MIN_ANGLE, descriptor);

    descriptor.description = DESCRIPTION_MAX_ANGLE;
    node->declare_parameter(PARAM_MAX_ANGLE, DEFAULT_MAX_ANGLE, descriptor);

    descriptor.description = DESCRIPTION_SAMPLE_COUNT;
    node->declare_parameter(PARAM_SAMPLE_COUNT, DEFAULT_SAMPLE_COUNT, descriptor);

    descriptor.description = DESCRIPTION_SAMPLING_FREQUENCY;
    node->declare_parameter(PARAM_SAMPLING_FREQUENCY, DEFAULT_SAMPLING_FREQUENCY, descriptor);
}

void LidarConfig::update_parameters(rclcpp::Node *node)
{
    this->range.first = node->get_parameter(PARAM_MIN_RANGE).as_double();
    this->range.second = node->get_parameter(PARAM_MAX_RANGE).as_double();
    this->angle.first = node->get_parameter(PARAM_MIN_ANGLE).as_double();
    this->angle.second = node->get_parameter(PARAM_MAX_ANGLE).as_double();
    this->sample_count = node->get_parameter(PARAM_SAMPLE_COUNT).as_int();
    this->sampling_frequency = node->get_parameter(PARAM_SAMPLING_FREQUENCY).as_double();
}

void LidarConfig::print_config(rclcpp::Node *node)
{
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_RANGE, this->range.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_RANGE, this->range.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_ANGLE, this->angle.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_ANGLE, this->angle.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_SAMPLE_COUNT, this->sample_count);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_SAMPLING_FREQUENCY,this->sampling_frequency);
    RCLCPP_INFO(node->get_logger(), "Scan step = %f", this->get_scan_step());
}