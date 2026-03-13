#include "ur_rtde_client/converter.hpp"

namespace ur_rtde_client
{
    std::unique_ptr<geometry_msgs::msg::TwistStamped>
    Converter::vector6dToTwistStamped(const std::vector<double> &data)
    {

        if (data.size() != 6 && data.size() != 3)
        {
            throw std::invalid_argument("vectordToTwistStamped: expected 3 or 6 elements");
        }

        auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "tool0";

        msg->twist.linear.x = data[0];
        msg->twist.linear.y = data[1];
        msg->twist.linear.z = data[2];

        if (data.size() == 6)
        {
            msg->twist.angular.x = data[3];
            msg->twist.angular.y = data[4];
            msg->twist.angular.z = data[5];
        }
        else
        {
            msg->twist.angular.x = 0.0;
            msg->twist.angular.y = 0.0;
            msg->twist.angular.z = 0.0;
        }
        return msg;
    }

    std::unique_ptr<geometry_msgs::msg::AccelStamped>
    Converter::vector6dToAccelStamped(const std::vector<double> &data)
    {

        if (data.size() != 6 && data.size() != 3)
        {
            throw std::invalid_argument("vectordToAccelStamped: expected 6 elements");
        }

        auto msg = std::make_unique<geometry_msgs::msg::AccelStamped>();
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "tool0";

        msg->accel.linear.x = data[0];
        msg->accel.linear.y = data[1];
        msg->accel.linear.z = data[2];
        msg->accel.angular.x = data[3];
        msg->accel.angular.y = data[4];
        msg->accel.angular.z = data[5];

        return msg;
    }

    std::unique_ptr<geometry_msgs::msg::PoseStamped>
    Converter::vector6dToPoseStamped(const std::vector<double> &data)
    {

        if (data.size() != 6)
        {
            throw std::invalid_argument("vectordToPoseStamped: expected 6 elements");
        }

        auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "tool0";

        msg->pose.position.x = data[0];
        msg->pose.position.y = data[1];
        msg->pose.position.z = data[2];

        const double rx = data[3], ry = data[4], rz = data[5];

        const double theta = sqrt(rx * rx + ry * ry + rz * rz);

        tf2::Quaternion q;
        if (theta < 1e-12)
        {
            q.setValue(0.0, 0.0, 0.0, 1.0);
        }
        else
        {
            const double ax = rx / theta;
            const double ay = ry / theta;
            const double az = rz / theta;
            q.setRotation(tf2::Vector3(ax, ay, az), theta);
        }

        msg->pose.orientation = tf2::toMsg(q);
        return msg;
    }

    std::unique_ptr<geometry_msgs::msg::InertiaStamped>
    Converter::vector6dToInertiaStamped(const std::vector<double> &data)
    {

        if (data.size() != 6)
        {
            throw std::invalid_argument("vectordToInertiaStamped: expected 6 elements");
        }

        auto msg = std::make_unique<geometry_msgs::msg::InertiaStamped>();
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "tool0";

        msg->inertia.ixx = data[0];
        msg->inertia.iyy = data[1];
        msg->inertia.izz = data[2];
        msg->inertia.ixy = data[3];
        msg->inertia.ixz = data[4];
        msg->inertia.iyz = data[5];

        return msg;
    }

    std::unique_ptr<geometry_msgs::msg::WrenchStamped>
    Converter::vector6dToWrenchStamped(const std::vector<double> &data)
    {

        if (data.size() != 6 && data.size() != 3)
        {
            throw std::invalid_argument("vectordToWrenchStamped: expected 6 elements");
        }

        auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "tool0";

        msg->wrench.force.x = data[0];
        msg->wrench.force.y = data[1];
        msg->wrench.force.z = data[2];
        msg->wrench.torque.x = data[3];
        msg->wrench.torque.y = data[4];
        msg->wrench.torque.z = data[5];

        return msg;
    }

    std::unique_ptr<geometry_msgs::msg::PointStamped>
    Converter::vector3dToPointStamped(const std::vector<double> &data)
    {

        if (data.size() != 3)
        {
            throw std::invalid_argument("vector3dToPointStamped: expected 3 elements");
        }

        auto msg = std::make_unique<geometry_msgs::msg::PointStamped>();
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "tool0";

        msg->point.x = data[0];
        msg->point.y = data[1];
        msg->point.z = data[2];

        return msg;
    }

    std::unique_ptr<geometry_msgs::msg::Vector3Stamped>
    Converter::vector3dToVector3Stamped(const std::vector<double> &data)
    {

        if (data.size() != 3)
        {
            throw std::invalid_argument("vector3dToVector3Stamped: expected 3 elements");
        }

        auto msg = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "tool0";

        msg->vector.x = data[0];
        msg->vector.y = data[1];
        msg->vector.z = data[2];

        return msg;
    }

    std::unique_ptr<sensor_msgs::msg::Temperature>
    Converter::doubleToTemperatureMsg(double value)
    {
        auto msg = std::make_unique<sensor_msgs::msg::Temperature>();
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "tool0";
        msg->temperature = value;
        return msg;
    }

    std::unique_ptr<example_interfaces::msg::UInt32>
    Converter::uint32ToMsg(uint32_t value)
    {
        auto msg = std::make_unique<example_interfaces::msg::UInt32>();
        msg->data = value;
        return msg;
    }

    std::unique_ptr<example_interfaces::msg::UInt8>
    Converter::uint8Tomsg(uint8_t value)
    {
        auto msg = std::make_unique<example_interfaces::msg::UInt8>();
        msg->data = value;
        return msg;
    }
    std::unique_ptr<example_interfaces::msg::Int32>
    Converter::int32ToMsg(int32_t value)
    {
        auto msg = std::make_unique<example_interfaces::msg::Int32>();
        msg->data = value;
        return msg;
    }

    std::unique_ptr<example_interfaces::msg::Float64>
    Converter::doubleToMsg(double value)
    {
        auto msg = std::make_unique<example_interfaces::msg::Float64>();
        msg->data = value;
        return msg;
    }

    std::unique_ptr<example_interfaces::msg::Bool>
    Converter::boolToMsg(bool value)
    {
        auto msg = std::make_unique<example_interfaces::msg::Bool>();
        msg->data = value;
        return msg;
    }

    std::unique_ptr<builtin_interfaces::msg::Time>
    Converter::doubleToTimeMsg(double value)
    {
        auto msg = std::make_unique<builtin_interfaces::msg::Time>();
        msg->sec = static_cast<int32_t>(value);
        msg->nanosec = static_cast<uint32_t>((value - msg->sec) * 1e9);
        return msg;
    }

    std::unique_ptr<ur_dashboard_msgs::msg::RobotMode>
    Converter::int32ToURRobotModeMsg(int32_t value)
    {
        auto msg = std::make_unique<ur_dashboard_msgs::msg::RobotMode>();
        msg->mode = int8_t(value);
        return msg;
    }
    std::unique_ptr<ur_dashboard_msgs::msg::SafetyMode>
    Converter::int32ToURSafetyModeMsg(int32_t value)
    {
        auto msg = std::make_unique<ur_dashboard_msgs::msg::SafetyMode>();
        msg->mode = u_int8_t(value);
        return msg;
    }

    std::unique_ptr<example_interfaces::msg::ByteMultiArray>
    Converter::uint32ToByteMultiArray(uint32_t value)
    {
        auto msg = std::make_unique<example_interfaces::msg::ByteMultiArray>();
        msg->data.reserve(32);

        for (size_t i = 0; i < 32; ++i)
        {
            uint8_t bit = static_cast<uint8_t>((value >> i) & 0x01);
            msg->data.push_back(bit);
        }

        return msg;
    }

    std::unique_ptr<example_interfaces::msg::ByteMultiArray>
    Converter::uint64ToByteMultiArray(uint64_t value)
    {
        auto msg = std::make_unique<example_interfaces::msg::ByteMultiArray>();
        msg->data.reserve(64);

        for (size_t i = 0; i < 64; ++i)
        {
            uint8_t bit = static_cast<uint8_t>((value >> i) & 0x01);
            msg->data.push_back(bit);
        }

        return msg;
    }

    std::unique_ptr<example_interfaces::msg::Float64MultiArray>
    Converter::vector6dToFloat64MultiArray(const std::vector<double> &data)
    {
        if (data.size() != 6)
        {
            throw std::invalid_argument("vector6dToFloat64MultiArray: expected 6 elements");
        }

        auto msg = std::make_unique<example_interfaces::msg::Float64MultiArray>();
        msg->data.reserve(6);

        for (const auto &value : data)
        {
            msg->data.push_back(value);
        }

        return msg;
    }

    std::unique_ptr<example_interfaces::msg::Int32MultiArray>
    Converter::vector6intToMsg(const std::vector<int32_t> &data)
    {

        auto msg = std::make_unique<example_interfaces::msg::Int32MultiArray>();
        msg->data.reserve(6);

        for (const auto &value : data)
        {
            msg->data.push_back(static_cast<int32_t>(value));
        }

        return msg;
    }

} // namespace ur_rtde_client