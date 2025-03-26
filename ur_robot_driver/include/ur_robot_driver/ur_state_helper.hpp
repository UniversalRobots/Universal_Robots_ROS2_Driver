#pragma once

#include "ur_robot_driver/hardware_interface.hpp"
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
        std::array<double, 18>& actual_dig_out_bits,
        std::array<double, 18>& actual_dig_in_bits,
        std::array<double, 11>& safety_status_bits,
        std::array<double, 4>& analog_io_types,
        std::array<double, 4>& robot_status_bits,
        std::array<double, 2>& tool_analog_input_types,
        std::array<double, 2>& tool_analog_input,
        std::array<double, 2>& standard_analog_input,
        std::array<double, 2>& standard_analog_output,
        double& tool_output_voltage,
        double& robot_mode,
        double& safety_mode,
        double& tool_mode,
        double& tool_output_current,
        double& tool_temperature,
        double& speed_scaling_combined,
        double& system_interface_initialized,
        double& robot_program_running,
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
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "digital_output_" + std::to_string(i), &actual_dig_out_bits[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "digital_input_" + std::to_string(i), &actual_dig_in_bits[i]));
        }

        for (size_t i = 0; i < 11; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "safety_status_bit_" + std::to_string(i), &safety_status_bits[i]));
        }

        for (size_t i = 0; i < 4; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "analog_io_type_" + std::to_string(i), &analog_io_types[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "robot_status_bit_" + std::to_string(i), &robot_status_bits[i]));
        }
       
        for (size_t i = 0; i < 2; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_analog_input_type_" + std::to_string(i), &tool_analog_input_types[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_analog_input_" + std::to_string(i), &tool_analog_input[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "standard_analog_input_" + std::to_string(i), &standard_analog_input[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "standard_analog_output_" + std::to_string(i), &standard_analog_output[i]));
        }

        // Other states
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_voltage", &tool_output_voltage));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "robot_mode", &robot_mode));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "safety_mode", &safety_mode));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_mode", &tool_mode));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_current", &tool_output_current));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_temperature", &tool_temperature));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "system_interface", "initialized", &system_interface_initialized));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "program_running", &robot_program_running));

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

    void update_non_double_values(
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
};

}  // namespace ur_robot_driver