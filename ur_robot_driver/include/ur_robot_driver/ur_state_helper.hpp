#pragma once

#include "ur_robot_driver/quaternion.hpp"
#include <bitset>
#include <vector>
#include <array>

namespace rtde = urcl::rtde_interface;

namespace ur_robot_driver
{

class URStateHelper
{
public:
    URStateHelper() = default; 

    static std::vector<hardware_interface::StateInterface> generate_state_interfaces(
        std::vector<std::string>& joint_names,
        urcl::vector6d_t& urcl_joint_positions,
        urcl::vector6d_t& urcl_joint_velocities,
        urcl::vector6d_t& urcl_joint_efforts,
        std::string& tf_prefix,
        std::vector<std::string>& sensor_names,
        urcl::vector6d_t& urcl_ft_sensor_measurements,
        std::array<double, 18>& actual_dig_out_bits_copy,
        std::array<double, 18>& actual_dig_in_bits_copy,
        std::array<double, 11>& safety_status_bits_copy,
        std::array<double, 4>& analog_io_types_copy,
        std::array<double, 4>& robot_status_bits_copy,
        std::array<double, 2>& tool_analog_input_types_copy,
        std::array<double, 2>& tool_analog_input,
        std::array<double, 2>& standard_analog_input,
        std::array<double, 2>& standard_analog_output,
        double& tool_output_voltage_copy,
        double& robot_mode_copy,
        double& safety_mode_copy,
        double& tool_mode_copy,
        double& tool_output_current,
        double& tool_temperature,
        double& speed_scaling_combined,
        double& system_interface_initialized,
        double& robot_program_running_copy,
        urcl::vector6d_t& urcl_tcp_pose,
        double& tcp_rotation_buffer_x,
        double& tcp_rotation_buffer_y,
        double& tcp_rotation_buffer_z,
        double& tcp_rotation_buffer_w,
        double& get_robot_software_version_major,
        double& get_robot_software_version_minor,
        double& get_robot_software_version_bugfix,
        double& get_robot_software_version_build)
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Joints
        for (size_t i = 0; i < joint_names.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names[i], hardware_interface::HW_IF_POSITION, &urcl_joint_positions[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names[i], hardware_interface::HW_IF_VELOCITY, &urcl_joint_velocities[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names[i], hardware_interface::HW_IF_EFFORT, &urcl_joint_efforts[i]));
        }

        // Speed scaling
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "speed_scaling", "speed_scaling_factor", &speed_scaling_combined));

        // Force-torque sensor
        for (const auto& sensor : sensor_names) {
            if (sensor == tf_prefix + "tcp_fts_sensor") {
                const std::vector<std::string> fts_names = {
                    "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"
                };
                for (uint j = 0; j < 6; ++j) {
                    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor, fts_names[j], &urcl_ft_sensor_measurements[j]));
                }
            }
        }

        // GPIO
        for (size_t i = 0; i < 18; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "digital_output_" + std::to_string(i), &actual_dig_out_bits_copy[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "digital_input_" + std::to_string(i), &actual_dig_in_bits_copy[i]));
        }

        for (size_t i = 0; i < 11; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "safety_status_bit_" + std::to_string(i), &safety_status_bits_copy[i]));
        }

        for (size_t i = 0; i < 4; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "analog_io_type_" + std::to_string(i), &analog_io_types_copy[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "robot_status_bit_" + std::to_string(i), &robot_status_bits_copy[i]));
        }
       
        for (size_t i = 0; i < 2; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_analog_input_type_" + std::to_string(i), &tool_analog_input_types_copy[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_analog_input_" + std::to_string(i), &tool_analog_input[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "standard_analog_input_" + std::to_string(i), &standard_analog_input[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "standard_analog_output_" + std::to_string(i), &standard_analog_output[i]));
        }

        // Other states
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_voltage", &tool_output_voltage_copy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "robot_mode", &robot_mode_copy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "safety_mode", &safety_mode_copy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_mode", &tool_mode_copy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_current", &tool_output_current));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_temperature", &tool_temperature));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "system_interface", "initialized", &system_interface_initialized));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "program_running", &robot_program_running_copy));

        // TCP pose
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.x", &urcl_tcp_pose[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.y", &urcl_tcp_pose[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.z", &urcl_tcp_pose[2]));

        // TCP rotation
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.x", &tcp_rotation_buffer_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.y", &tcp_rotation_buffer_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.z", &tcp_rotation_buffer_z));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.w", &tcp_rotation_buffer_w));

        // Software version
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "get_robot_software_version", "get_version_major", &get_robot_software_version_major));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "get_robot_software_version", "get_version_minor", &get_robot_software_version_minor));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "get_robot_software_version", "get_version_bugfix", &get_robot_software_version_bugfix));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "get_robot_software_version", "get_version_build", &get_robot_software_version_build));

        return state_interfaces;
    }

    static void process_state_data(
        std::unique_ptr<rtde::DataPackage>& data_pkg,
        urcl::vector6d_t& urcl_joint_positions,
        urcl::vector6d_t& urcl_joint_velocities,
        urcl::vector6d_t& urcl_joint_efforts,
        double& target_speed_fraction,
        double& speed_scaling,
        uint32_t& runtime_state,
        urcl::vector6d_t& urcl_ft_sensor_measurements,
        urcl::vector6d_t& urcl_tcp_pose,
        std::array<double, 2>& standard_analog_input,
        std::array<double, 2>& standard_analog_output,
        std::array<double, 2>& tool_analog_input,
        double& tool_output_current,
        double& tool_temperature,
        std::bitset<4>& robot_status_bits,
        std::array<double, 4>& robot_status_bits_copy,
        std::bitset<11>& safety_status_bits,
        std::array<double, 11>& safety_status_bits_copy,
        std::bitset<18>& actual_dig_in_bits,
        std::array<double, 18>& actual_dig_in_bits_copy,
        std::bitset<4>& analog_io_types,
        std::array<double, 4>& analog_io_types_copy,
        std::bitset<2>& tool_analog_input_types,
        std::array<double, 2>& tool_analog_input_types_copy,
        tf2::Quaternion& tcp_rotation_quat,
        Quaternion& tcp_rotation_buffer,
        tf2::Vector3& tcp_force,
        tf2::Vector3& tcp_torque,
        std::bitset<18>& actual_dig_out_bits,
        std::array<double, 18>& actual_dig_out_bits_copy,
        int32_t& tool_output_voltage,
        double& tool_output_voltage_copy,
        int32_t& robot_mode,
        double& robot_mode_copy,
        int32_t& safety_mode,
        double& safety_mode_copy,
        uint32_t& tool_mode,
        double& tool_mode_copy,
        bool& initialized,
        double& system_interface_initialized,
        bool& robot_program_running,
        double& robot_program_running_copy)
    {

        if (!data_pkg) {
            return;
        }

        read_data(data_pkg, "actual_q", urcl_joint_positions);
        read_data(data_pkg, "actual_qd", urcl_joint_velocities);
        read_data(data_pkg, "actual_current", urcl_joint_efforts);
        read_data(data_pkg, "target_speed_fraction", target_speed_fraction);
        read_data(data_pkg, "speed_scaling", speed_scaling);
        read_data(data_pkg, "runtime_state", runtime_state);
        read_data(data_pkg, "actual_TCP_force", urcl_ft_sensor_measurements);
        read_data(data_pkg, "actual_TCP_pose", urcl_tcp_pose);
        read_data(data_pkg, "standard_analog_input0", standard_analog_input[0]);
        read_data(data_pkg, "standard_analog_input1", standard_analog_input[1]);
        read_data(data_pkg, "standard_analog_output0", standard_analog_output[0]);
        read_data(data_pkg, "standard_analog_output1", standard_analog_output[1]);
        read_data(data_pkg, "tool_mode", tool_mode);
        read_data(data_pkg, "tool_analog_input0", tool_analog_input[0]);
        read_data(data_pkg, "tool_analog_input1", tool_analog_input[1]);
        read_data(data_pkg, "tool_output_voltage", tool_output_voltage);
        read_data(data_pkg, "tool_output_current", tool_output_current);
        read_data(data_pkg, "tool_temperature", tool_temperature);
        read_data(data_pkg, "robot_mode", robot_mode);
        read_data(data_pkg, "safety_mode", safety_mode);

        read_bitset_data<uint32_t>(data_pkg, "robot_status_bits", robot_status_bits);
        read_bitset_data<uint32_t>(data_pkg, "safety_status_bits", safety_status_bits);
        read_bitset_data<uint64_t>(data_pkg, "actual_digital_input_bits", actual_dig_in_bits);
        read_bitset_data<uint64_t>(data_pkg, "actual_digital_output_bits", actual_dig_out_bits);
        read_bitset_data<uint32_t>(data_pkg, "analog_io_types", analog_io_types);
        read_bitset_data<uint32_t>(data_pkg, "tool_analog_input_types", tool_analog_input_types);

        extract_tool_pose(urcl_tcp_pose, tcp_rotation_quat, tcp_rotation_buffer);
        transform_force_torque(tcp_rotation_quat, tcp_force, tcp_torque, urcl_ft_sensor_measurements);
        update_non_double_values(
            actual_dig_out_bits, actual_dig_out_bits_copy,
            actual_dig_in_bits, actual_dig_in_bits_copy,
            safety_status_bits, safety_status_bits_copy,
            analog_io_types, analog_io_types_copy,
            robot_status_bits, robot_status_bits_copy,
            tool_analog_input_types, tool_analog_input_types_copy,
            tool_output_voltage, tool_output_voltage_copy,
            robot_mode, robot_mode_copy,
            safety_mode, safety_mode_copy,
            tool_mode, tool_mode_copy,
            initialized, system_interface_initialized,
            robot_program_running, robot_program_running_copy);
    }

private:
    static void update_non_double_values(
        const std::bitset<18>& actual_dig_out_bits, std::array<double, 18>& actual_dig_out_bits_copy,
        const std::bitset<18>& actual_dig_in_bits, std::array<double, 18>& actual_dig_in_bits_copy,
        const std::bitset<11>& safety_status_bits, std::array<double, 11>& safety_status_bits_copy,
        const std::bitset<4>& analog_io_types, std::array<double, 4>& analog_io_types_copy,
        const std::bitset<4>& robot_status_bits, std::array<double, 4>& robot_status_bits_copy,
        const std::bitset<2>& tool_analog_input_types, std::array<double, 2>& tool_analog_input_types_copy,
        uint8_t tool_output_voltage, double& tool_output_voltage_copy,
        uint8_t robot_mode, double& robot_mode_copy,
        uint8_t safety_mode, double& safety_mode_copy,
        uint8_t tool_mode, double& tool_mode_copy,
        bool initialized, double& system_interface_initialized,
        bool robot_program_running, double& robot_program_running_copy)
    {
        for (size_t i = 0; i < 18; ++i) {
            actual_dig_out_bits_copy[i] = static_cast<double>(actual_dig_out_bits[i]);
            actual_dig_in_bits_copy[i] = static_cast<double>(actual_dig_in_bits[i]);
        }

        for (size_t i = 0; i < 11; ++i) {
            safety_status_bits_copy[i] = static_cast<double>(safety_status_bits[i]);
        }

        for (size_t i = 0; i < 4; ++i) {
            analog_io_types_copy[i] = static_cast<double>(analog_io_types[i]);
            robot_status_bits_copy[i] = static_cast<double>(robot_status_bits[i]);
        }

        for (size_t i = 0; i < 2; ++i) {
            tool_analog_input_types_copy[i] = static_cast<double>(tool_analog_input_types[i]);
        }

        tool_output_voltage_copy = static_cast<double>(tool_output_voltage);
        robot_mode_copy = static_cast<double>(robot_mode);
        safety_mode_copy = static_cast<double>(safety_mode);
        tool_mode_copy = static_cast<double>(tool_mode);
        system_interface_initialized = initialized ? 1.0 : 0.0;
        robot_program_running_copy = robot_program_running ? 1.0 : 0.0;
    }


    template <typename T>
    static void read_data(const std::unique_ptr<rtde::DataPackage>& data_pkg, const std::string& var_name, T& data)
    {
        if (!data_pkg->getData(var_name, data)) {
            // This throwing should never happen unless misconfigured
            std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
            throw std::runtime_error(error_msg);
        }
    }

    template <typename T, size_t N>
    static void read_bitset_data(const std::unique_ptr<rtde::DataPackage>& data_pkg, const std::string& var_name, std::bitset<N>& data)
    {
        if (!data_pkg->getData<T, N>(var_name, data)) {
            // This throwing should never happen unless misconfigured
            std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
            throw std::runtime_error(error_msg);
        }
    }

    static void extract_tool_pose(
        urcl::vector6d_t& urcl_tcp_pose,
        tf2::Quaternion& tcp_rotation_quat,
        Quaternion& tcp_rotation_buffer)
    {
        // imported from ROS1 driver hardware_interface.cpp#L911-L928
        double tcp_angle =
            std::sqrt(std::pow(urcl_tcp_pose[3], 2) + std::pow(urcl_tcp_pose[4], 2) + std::pow(urcl_tcp_pose[5], 2));

        tf2::Vector3 rotation_vec(urcl_tcp_pose[3], urcl_tcp_pose[4], urcl_tcp_pose[5]);
        if (tcp_angle > 1e-16) {
            tcp_rotation_quat.setRotation(rotation_vec.normalized(), tcp_angle);
        } else {
            tcp_rotation_quat.setValue(0.0, 0.0, 0.0, 1.0);  // default Quaternion is 0,0,0,0 which is invalid
        }
        tcp_rotation_buffer.set(tcp_rotation_quat);
    }

    static void transform_force_torque(
        tf2::Quaternion& tcp_rotation_quat,
        tf2::Vector3& tcp_force,
        tf2::Vector3& tcp_torque,
        urcl::vector6d_t& urcl_ft_sensor_measurements)
    {
        // imported from ROS1 driver - hardware_interface.cpp#L867-L876
        tcp_force.setValue(urcl_ft_sensor_measurements[0], urcl_ft_sensor_measurements[1],
                            urcl_ft_sensor_measurements[2]);
        tcp_torque.setValue(urcl_ft_sensor_measurements[3], urcl_ft_sensor_measurements[4],
                            urcl_ft_sensor_measurements[5]);

        tcp_force = tf2::quatRotate(tcp_rotation_quat.inverse(), tcp_force);
        tcp_torque = tf2::quatRotate(tcp_rotation_quat.inverse(), tcp_torque);

        urcl_ft_sensor_measurements = { tcp_force.x(),  tcp_force.y(),  tcp_force.z(),
                                        tcp_torque.x(), tcp_torque.y(), tcp_torque.z() };
    }
};

}  // namespace ur_robot_driver