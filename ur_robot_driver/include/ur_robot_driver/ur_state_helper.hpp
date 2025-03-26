#pragma once

#include "ur_robot_driver/hardware_interface.hpp"
#include <bitset>
#include <vector>
#include <array>

namespace ur_robot_driver
{

class URStateHelper
{
public:
    URStateHelper() = default; 

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

};

}  // namespace ur_robot_driver