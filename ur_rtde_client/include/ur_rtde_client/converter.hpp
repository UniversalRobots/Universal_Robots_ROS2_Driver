#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <example_interfaces/msg/u_int8.hpp>
#include <example_interfaces/msg/u_int32.hpp>
#include <example_interfaces/msg/u_int64.hpp>
#include <example_interfaces/msg/int32.hpp>
#include <example_interfaces/msg/byte_multi_array.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <example_interfaces/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "ur_client_library/rtde/rtde_client.h"
#include <ur_dashboard_msgs/msg/robot_mode.hpp>
#include <ur_dashboard_msgs/msg/safety_mode.hpp>

namespace ur_rtde_client
{
    /**
     * @brief Utility class to convert raw RTDE-like data into ROS 2 messages.
     */
    class Converter
    {
    public:
        static std::unique_ptr<geometry_msgs::msg::TwistStamped>
        vector6dToTwistStamped(const std::vector<double> &data);

        static std::unique_ptr<example_interfaces::msg::UInt32>
        uint32ToMsg(uint32_t value);

        static std::unique_ptr<example_interfaces::msg::Float64>
        doubleToMsg(double value);

        static std::unique_ptr<example_interfaces::msg::Float64MultiArray>
        vector6dToFloat64MultiArray(const std::vector<double> &data);

        static std::unique_ptr<geometry_msgs::msg::WrenchStamped>
        vector6dToWrenchStamped(const std::vector<double> &data);

        static std::unique_ptr<geometry_msgs::msg::PoseStamped>
        vector6dToPoseStamped(const std::vector<double> &data);

        static std::unique_ptr<geometry_msgs::msg::AccelStamped>
        vector6dToAccelStamped(const std::vector<double> &data);

        static std::unique_ptr<example_interfaces::msg::Int32>
        int32ToMsg(int32_t value);

        static std::unique_ptr<geometry_msgs::msg::Vector3Stamped>
        vector3dToVector3Stamped(const std::vector<double> &data);

        static std::unique_ptr<geometry_msgs::msg::PointStamped>
        vector3dToPointStamped(const std::vector<double> &data);

        static std::unique_ptr<builtin_interfaces::msg::Time>
        doubleToTimeMsg(double value);

        static std::unique_ptr<example_interfaces::msg::ByteMultiArray>
        uint32ToByteMultiArray(uint32_t value);

        static std::unique_ptr<example_interfaces::msg::ByteMultiArray>
        uint64ToByteMultiArray(uint64_t value);

        static std::unique_ptr<ur_dashboard_msgs::msg::RobotMode>
        int32ToURRobotModeMsg(int32_t value);

        static std::unique_ptr<ur_dashboard_msgs::msg::SafetyMode>
        int32ToURSafetyModeMsg(int32_t value);

        static std::unique_ptr<geometry_msgs::msg::InertiaStamped>
        vector6dToInertiaStamped(const std::vector<double> &data);

        static std::unique_ptr<sensor_msgs::msg::Temperature>
        doubleToTemperatureMsg(double value);

        static std::unique_ptr<example_interfaces::msg::UInt8>
        uint8Tomsg(uint8_t value);

        static std::unique_ptr<example_interfaces::msg::Int32MultiArray>
        vector6intToMsg(const std::vector<int32_t> &data);

        static std::unique_ptr<example_interfaces::msg::Bool>
        boolToMsg(bool value);
    };

} // namespace ur_rtde_client