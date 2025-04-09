// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>

#include "ur_robot_motion_primitives_driver/motion_primitives_ur_driver.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "motion_primitives_forward_controller/motion_type.hpp"

namespace ur_robot_motion_primitives_driver
{

MotionPrimitivesUrDriver::~MotionPrimitivesUrDriver()
{
  async_thread_shutdown_ = true;
  if (async_command_thread_ && async_command_thread_->joinable()) {
    async_command_thread_->join();
    async_command_thread_.reset();
  }
  if (async_stop_motion_thread_ && async_stop_motion_thread_->joinable()) {
    async_stop_motion_thread_->join();
    async_stop_motion_thread_.reset();
  }

  ur_driver_.reset();
}

hardware_interface::CallbackReturn MotionPrimitivesUrDriver::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Initializing Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Failed to initialize SystemInterface");
    return CallbackReturn::ERROR;
  }

  info_ = info;

  // Initialize the state helper
  state_helper_.initialize();

  // initialize
  initialized_ = false;
  async_thread_shutdown_ = false;
  new_command_available_ = false;
  rtde_comm_has_been_started_ = false;

  // Resize hardware state and command vectors, initializing them with NaN values.
  // TODO (mathias31415): is there a better way than hardcoding the size of the vectors? --> read the number of state and command interfaces somehow?
  hw_mo_prim_states_.resize(1, std::numeric_limits<double>::quiet_NaN());     // 1 status
  hw_mo_prim_commands_.resize(25, std::numeric_limits<double>::quiet_NaN());  // motion_type + 6 joints + 2*7 positions + blend_radius + velocity + acceleration + move_time

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionPrimitivesUrDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Configuring Hardware Interface");

  // The robot's IP address.
  const std::string robot_ip = info_.hardware_parameters["robot_ip"];
  
  // Path to the urscript code that will be sent to the robot
  const std::string script_filename = info_.hardware_parameters["script_filename"];
  
  // Path to the file containing the recipe used for requesting RTDE outputs.
  const std::string output_recipe_filename = info_.hardware_parameters["output_recipe_filename"];
  
  // Path to the file containing the recipe used for requesting RTDE inputs.
  const std::string input_recipe_filename = info_.hardware_parameters["input_recipe_filename"];
  
  // Start robot in headless mode. This does not require the 'External Control' URCap to be running
  // on the robot, but this will send the URScript to the robot directly. On e-Series robots this
  // requires the robot to run in 'remote-control' mode.
  const bool headless_mode = (info_.hardware_parameters["headless_mode"] == "true") || (info_.hardware_parameters["headless_mode"] == "True");
  
  // Port that will be opened to communicate between the driver and the robot controller.
  const int reverse_port = stoi(info_.hardware_parameters["reverse_port"]);

  // The driver will offer an interface to receive the program's URScript on this port.
  const int script_sender_port = stoi(info_.hardware_parameters["script_sender_port"]);

  // The ip address of the host the driver runs on
  std::string reverse_ip = info_.hardware_parameters["reverse_ip"];
  if (reverse_ip == "0.0.0.0") {
    reverse_ip = "";
  }

  // Port (on the host pc) of the trajectory interface
  const int trajectory_port = stoi(info_.hardware_parameters["trajectory_port"]);

  // Port (on the host PC) that will be used to forward script commands from the driver to the robot
  const int script_command_port = stoi(info_.hardware_parameters["script_command_port"]);

  // Enables non_blocking_read mode. Should only be used with combined_robot_hw. Disables error generated when read
  // returns without any data, sets the read timeout to zero, and synchronises read/write operations. Enabling this when
  // not used with combined_robot_hw can suppress important errors and affect real-time performance.
  non_blocking_read_ = (info_.hardware_parameters["non_blocking_read"] == "true") ||
                       (info_.hardware_parameters["non_blocking_read"] == "True");

  // Specify gain for servoing to position in joint space.
  // A higher gain can sharpen the trajectory.
  const int servoj_gain = stoi(info_.hardware_parameters["servoj_gain"]);
  // Specify lookahead time for servoing to position in joint space.
  // A longer lookahead time can smooth the trajectory.
  const double servoj_lookahead_time = stod(info_.hardware_parameters["servoj_lookahead_time"]);

  // Hash of the calibration reported by the robot. This is used for validating the robot
  // description is using the correct calibration. If the robot's calibration doesn't match this
  // hash, an error will be printed. You can use the robot as usual, however Cartesian poses of the
  // endeffector might be inaccurate. See the "ur_calibration" package on help how to generate your
  // own hash matching your actual robot.
  const std::string calibration_checksum = info_.hardware_parameters["kinematics/hash"];

  // Obtain the tf_prefix which is needed for the logging handler so that log messages from different arms are
  // distiguishable in the log
  const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Initializing driver...");
  try {
    rtde_comm_has_been_started_ = false;
    urcl::UrDriverConfiguration driver_config;
    driver_config.robot_ip = robot_ip;
    driver_config.script_file = script_filename;
    driver_config.output_recipe_file = output_recipe_filename;
    driver_config.input_recipe_file = input_recipe_filename;
    driver_config.headless_mode = headless_mode;
    driver_config.reverse_port = static_cast<uint32_t>(reverse_port);
    driver_config.script_sender_port = static_cast<uint32_t>(script_sender_port);
    driver_config.trajectory_port = static_cast<uint32_t>(trajectory_port);
    driver_config.script_command_port = static_cast<uint32_t>(script_command_port);
    driver_config.reverse_ip = reverse_ip;
    driver_config.servoj_gain = static_cast<uint32_t>(servoj_gain);
    driver_config.servoj_lookahead_time = servoj_lookahead_time;
    driver_config.non_blocking_read = non_blocking_read_;
    driver_config.tool_comm_setup = nullptr;
    driver_config.handle_program_state =
        std::bind(&MotionPrimitivesUrDriver::handleRobotProgramState, this, std::placeholders::_1);
    ur_driver_ = std::make_shared<urcl::UrDriver>(driver_config);
  } catch (urcl::ToolCommNotAvailable& e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("MotionPrimitivesUrDriver"), "See parameter use_tool_communication");

    return hardware_interface::CallbackReturn::ERROR;
  } catch (urcl::UrException& e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("MotionPrimitivesUrDriver"), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Timeout before the reverse interface will be dropped by the robot
  receive_timeout_ = urcl::RobotReceiveTimeout::sec(std::stof(info_.hardware_parameters["robot_receive_timeout"]));

  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Calibration checksum: '%s'.",
              calibration_checksum.c_str());
  // check calibration
  // https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/c3378599d5fa73a261328b326392e847f312ab6b/ur_robot_driver/src/hardware_interface.cpp#L296-L309
  if (ur_driver_->checkCalibration(calibration_checksum)) {
    RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Calibration checked successfully.");
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotionPrimitivesUrDriver"),

                        "The calibration parameters of the connected robot don't match the ones from the given "
                        "kinematics config file. Please be aware that this can lead to critical inaccuracies of tcp "
                        "positions. Use the ur_calibration tool to extract the correct calibration from the robot and "
                        "pass that into the description. See "
                        "[https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_calibration/"
                        "README.md] for details.");
  }

  // ur_driver_->resetRTDEClient(output_recipe_filename, input_recipe_filename, 125.0, false);  // set rtde frequency to 125 Hz

  // Export version information to state interfaces
  urcl::VersionInformation version_info = ur_driver_->getVersion();
  state_helper_.setRobotSoftwareVersion(version_info);

  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Initializing InstructionExecutor");
  instruction_executor_ = std::make_shared<urcl::InstructionExecutor>(ur_driver_);

  // Start async thread for sending commands
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Starting async thread");
  async_command_thread_ = std::make_unique<std::thread>(&MotionPrimitivesUrDriver::asyncCommandThread, this);
  async_stop_motion_thread_ = std::make_unique<std::thread>(&MotionPrimitivesUrDriver::asyncStopMotionThread, this);

  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "System successfully started!");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotionPrimitivesUrDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Exporting State Interfaces");

  // extract joint names
  std::vector<std::string> joint_names;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_names.push_back(info_.joints[i].name);
  }

  // extract sensor names
  std::vector<std::string> sensor_names;
  for (size_t i = 0; i < info_.sensors.size(); ++i) {
    sensor_names.push_back(info_.sensors[i].name);
  }
  // state interfaces from the URPositionHardwareInterface
  state_interfaces = state_helper_.generate_state_interfaces(joint_names, info_.hardware_parameters.at("tf_prefix"), sensor_names);

  // State for execution_status
  state_interfaces.emplace_back(hardware_interface::StateInterface("motion_primitive", "execution_status", &hw_mo_prim_states_[0]));

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> MotionPrimitivesUrDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Exporting Command Interfaces");

  // Command for motion type (motion_type)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "motion_type", &hw_mo_prim_commands_[0]));
  // Joint position commands (q1, q2, ..., q6)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q1", &hw_mo_prim_commands_[1]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q2", &hw_mo_prim_commands_[2]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q3", &hw_mo_prim_commands_[3]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q4", &hw_mo_prim_commands_[4]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q5", &hw_mo_prim_commands_[5]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q6", &hw_mo_prim_commands_[6]));
  // Position commands (pos_x, pos_y, pos_z, pos_qx, pos_qy, pos_qz, pos_qz)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_x", &hw_mo_prim_commands_[7]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_y", &hw_mo_prim_commands_[8]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_z", &hw_mo_prim_commands_[9]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qx", &hw_mo_prim_commands_[10]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qy", &hw_mo_prim_commands_[11]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qz", &hw_mo_prim_commands_[12]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qw", &hw_mo_prim_commands_[13]));
  // Via Position commands for circula motion
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_x", &hw_mo_prim_commands_[14]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_y", &hw_mo_prim_commands_[15]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_z", &hw_mo_prim_commands_[16]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qx", &hw_mo_prim_commands_[17]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qy", &hw_mo_prim_commands_[18]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qz", &hw_mo_prim_commands_[19]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qw", &hw_mo_prim_commands_[20]));
  // Other command parameters (blend_radius, velocity, acceleration, move_time)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "blend_radius", &hw_mo_prim_commands_[21]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "velocity", &hw_mo_prim_commands_[22]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "acceleration", &hw_mo_prim_commands_[23]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "move_time", &hw_mo_prim_commands_[24]));

  return command_interfaces;
}


hardware_interface::CallbackReturn MotionPrimitivesUrDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Activating Hardware Interface");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionPrimitivesUrDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Deactivating Hardware Interface");
  ur_driver_.reset();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotionPrimitivesUrDriver::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!rtde_comm_has_been_started_) {
    ur_driver_->startRTDECommunication();
    rtde_comm_has_been_started_ = true;
  }

  std::unique_ptr<urcl::rtde_interface::DataPackage> data_pkg = ur_driver_->getDataPackage();

  if (data_pkg) {
    state_helper_.processStateData(data_pkg, initialized_, robot_program_running_);
  }

  // Update the state interface with the current execution status
  hw_mo_prim_states_[0] = current_execution_status_;    // 0=idle, 1=executing, 2=success, 3=error

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type MotionPrimitivesUrDriver::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Check if we have a new command
  if (!std::isnan(hw_mo_prim_commands_[0])) {
    if (hw_mo_prim_commands_[0] == MotionType::STOP_MOTION) {
      // Stop command received
      std::lock_guard<std::mutex> guard(stop_mutex_);
      if (!new_stop_available_) {
        new_stop_available_ = true;
        // Reset command interfaces
        std::fill(hw_mo_prim_commands_.begin(), hw_mo_prim_commands_.end(), std::numeric_limits<double>::quiet_NaN());
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Command of type: %f recived", hw_mo_prim_commands_[0]);
      std::lock_guard<std::mutex> guard(command_mutex_);
      if (!new_command_available_) {
        // Copy command to thread-safe buffer
        pending_command_ = hw_mo_prim_commands_;
        new_command_available_ = true;
        // Reset command interfaces
        std::fill(hw_mo_prim_commands_.begin(), hw_mo_prim_commands_.end(), std::numeric_limits<double>::quiet_NaN());
      }
    }
  }

  // Send keepalive if current_execution_status_ in not EXECUTING
  if (ur_driver_ && current_execution_status_ != ExecutionState::EXECUTING) {
    ur_driver_->writeKeepalive();
  }


  return hardware_interface::return_type::OK;
}

void MotionPrimitivesUrDriver::asyncStopMotionThread()
{
  while (!async_thread_shutdown_) {
    if (new_stop_available_) {
      {
        std::lock_guard<std::mutex> guard(stop_mutex_);
        new_stop_available_ = false;
      }
      // Stop the motion
      RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "[asyncStopMotionThread] New stop command available");
      processStopCommand();
    }

    // Small sleep to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "[asyncStopMotionThread] Exiting");
}

void MotionPrimitivesUrDriver::processStopCommand()
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Received STOP command");
  // RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Trajectory running: %d", instruction_executor_->isTrajectoryRunning());
  
  // delete motion sequence
  build_motion_sequence_ = false;
  motion_sequence_.clear();

  if (instruction_executor_->isTrajectoryRunning()) {
    RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Stopping motion ...");
    if(!instruction_executor_->cancelMotion()) {
      RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Failed to stop motion");
      current_execution_status_ = ExecutionState::ERROR;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Motion stopped successfully");
      current_execution_status_ = ExecutionState::IDLE;
    }
  }
  else {
    RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "No motion to stop");
    current_execution_status_ = ExecutionState::IDLE;
  }
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), " [processStopCommand] After executing stop: current_execution_status_ = %d", current_execution_status_.load());
}

void MotionPrimitivesUrDriver::asyncCommandThread()
{
  while (!async_thread_shutdown_) {
    // Check for new commands
    if (new_command_available_) {
      // RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "[asyncCommandThread] New command available");
      std::vector<double> current_command;
      {
        std::lock_guard<std::mutex> guard(command_mutex_);
        current_command = pending_command_;
        new_command_available_ = false;
      }

      // Process the command
      processMotionCommand(current_command);
    }

    // Small sleep to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "[asyncCommandThread] Exiting");
}

void MotionPrimitivesUrDriver::processMotionCommand(const std::vector<double>& command)
{
  if (command.empty() || std::isnan(command[0])) {
    return;
  }
  current_execution_status_ = ExecutionState::EXECUTING;

  double motion_type = command[0];
  
  double velocity, acceleration, move_time; 

  // TODO (mathias31415): check if nan needs to be handeled
  double blend_radius = command[21];

  try {
    switch (static_cast<uint8_t>(motion_type)) {
      case MotionType::MOTION_SEQUENCE_START: {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Received motion type: MOTION_SEQUENCE_START");
        build_motion_sequence_ = true;  // set flag to put all following commands into the motion sequence
        motion_sequence_.clear();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        current_execution_status_ = ExecutionState::IDLE;   // set to IDLE to allow controller to send new commands
        break;
      }

      case MotionType::MOTION_SEQUENCE_END: {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Received motion type: MOTION_SEQUENCE_END");
        build_motion_sequence_ = false;
        bool success = instruction_executor_->executeMotion(motion_sequence_);
        current_execution_status_ = success ? ExecutionState::SUCCESS : ExecutionState::ERROR;
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), " [processMotionCommand] After executing motion sequence: current_execution_status_ = %d", current_execution_status_.load());
        motion_sequence_.clear();
        break;
      }

      case MotionType::LINEAR_JOINT: { // moveJ
        // Check if joint positions are valid
        for (int i = 1; i <= 6; ++i) {
            if (std::isnan(command[i])) {
                RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Invalid motion command: joint positions contain NaN values");
                current_execution_status_ = ExecutionState::ERROR;
                return;
            }
        }
        urcl::vector6d_t joint_positions = {command[1], command[2], command[3], command[4], command[5], command[6]};
        
        // Get move_time OR (velocity AND acceleration)
        if (!getMovetimeOrVelocityAndAcceleration(command, velocity, acceleration, move_time)) {
          RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Invalid move_time, velocity or acceleration values");
          current_execution_status_ = ExecutionState::ERROR;
          return;
        }

        
        // Check if the command is part of a motion sequence or a single command
        if (build_motion_sequence_) {   // Add command to motion sequence
          motion_sequence_.emplace_back(
            std::make_shared<urcl::control::MoveJPrimitive>(
                joint_positions, blend_radius, std::chrono::milliseconds(static_cast<int>(move_time * 1000)), acceleration, velocity));        
          RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), 
                "Added moveJ to motion sequence with joint positions: [%f, %f, %f, %f, %f, %f], velocity: %f, acceleration: %f, move_time: %f, blend_radius: %f", 
                joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4], joint_positions[5],
                velocity, acceleration, move_time, blend_radius);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          current_execution_status_ = ExecutionState::IDLE;   // set to IDLE to allow controller to send new commands
          return;
        }
        else{ // execute single primitive directly
          RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), 
                "Executing moveJ with joint positions: [%f, %f, %f, %f, %f, %f], velocity: %f, acceleration: %f, move_time: %f, blend_radius: %f", 
                joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4], joint_positions[5]
                , velocity, acceleration, move_time, blend_radius);
          bool success = instruction_executor_->moveJ(joint_positions, acceleration, velocity, move_time, blend_radius);
          current_execution_status_ = success ? ExecutionState::SUCCESS : ExecutionState::ERROR;
          RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), " [processMotionCommand] After executing moveJ: current_execution_status_ = %d", current_execution_status_.load());
          return;
        }
        break;
      }

      case MotionType::LINEAR_CARTESIAN: { // moveL
        // Check if pose values (position and quaternion) are valid
        for (int i = 7; i <= 13; ++i) {
            if (std::isnan(command[i])) {
                RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Invalid motion command: pose contains NaN values");
                current_execution_status_ = ExecutionState::ERROR;
                return;
            }
        }
        double rx, ry, rz;
        quaternionToEuler(command[10], command[11], command[12], command[13], rx, ry, rz);
        urcl::Pose pose = { command[7], command[8], command[9], rx, ry, rz};

        // Get move_time OR (velocity AND acceleration)
        if (!getMovetimeOrVelocityAndAcceleration(command, velocity, acceleration, move_time)) {
          RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Invalid move_time, velocity or acceleration values");
          current_execution_status_ = ExecutionState::ERROR;
          return;
        }

        // Check if the command is part of a motion sequence or a single command
        if (build_motion_sequence_) {   // Add command to motion sequence
          motion_sequence_.emplace_back(
            std::make_shared<urcl::control::MoveLPrimitive>(
                pose, blend_radius, std::chrono::milliseconds(static_cast<int>(move_time * 1000)), acceleration, velocity));
          RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), 
                "Added  moveL to motion sequence with pose: [%f, %f, %f, %f, %f, %f], velocity: %f, acceleration: %f, move_time: %f, blend_radius: %f", 
                pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz, velocity, acceleration, move_time, blend_radius);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          current_execution_status_ = ExecutionState::IDLE;   // set to IDLE to allow controller to send new commands
          return;
        } 
        else{ // execute single primitive directly
          RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), 
                  "Executing moveL with pose: [%f, %f, %f, %f, %f, %f], velocity: %f, acceleration: %f, move_time: %f, blend_radius: %f", 
                  pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz, velocity, acceleration, move_time, blend_radius);
          bool success = instruction_executor_->moveL(pose, acceleration, velocity, move_time, blend_radius);
          current_execution_status_ = success ? ExecutionState::SUCCESS : ExecutionState::ERROR;
          RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), " [processMotionCommand] After executing moveL: current_execution_status_ = %d", current_execution_status_.load());
          return;
        }
        break;
      }

      case MotionType::CIRCULAR_CARTESIAN: { //CIRC
        // Check if pose values (position and quaternion) are valid
        for (int i = 7; i <= 20; ++i) {
          if (std::isnan(command[i])) {
              RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Invalid motion command: pose contains NaN values");
              current_execution_status_ = ExecutionState::ERROR;
              return;
          }
        }

        int32_t mode = 0; // 0: Unconstrained mode, 1: Fixed mode (https://tools.pages.cba.mit.edu/Universal_Robotics_UR10_Robot_Arm/scriptManual-3.5.4.pdf )

        // Get velocity and acceleration)
        if (!getMovetimeOrVelocityAndAcceleration(command, velocity, acceleration, move_time)) {
          RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Invalid move_time, velocity or acceleration values");
          current_execution_status_ = ExecutionState::ERROR;
          return;
        }

        double via_rx, via_ry, via_rz;
        quaternionToEuler(command[17], command[18], command[19], command[20], via_rx, via_ry, via_rz);
        urcl::Pose via_pose = { command[14], command[15], command[16], via_rx, via_ry, via_rz };

        double goal_rx, goal_ry, goal_rz;
        quaternionToEuler(command[10], command[11], command[12], command[13], goal_rx, goal_ry, goal_rz);
        urcl::Pose goal_pose = { command[7], command[8], command[9], goal_rx, goal_ry, goal_rz };
        
        // Check if the command is part of a motion sequence or a single command
        if (build_motion_sequence_) {   // Add command to motion sequence
          motion_sequence_.emplace_back(
            std::make_shared<urcl::control::MoveCPrimitive>(
                via_pose, goal_pose, blend_radius, acceleration, velocity, mode));
          RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), 
                "Added  moveC to motion sequence with via_pose: [%f, %f, %f, %f, %f, %f], goal_pose: [%f, %f, %f, %f, %f, %f], velocity: %f, acceleration: %f, blend_radius: %f, mode: %d", 
                via_pose.x, via_pose.y, via_pose.z, via_pose.rx, via_pose.ry, via_pose.rz,
                goal_pose.x, goal_pose.y, goal_pose.z, goal_pose.rx, goal_pose.ry, goal_pose.rz,
                velocity, acceleration, blend_radius, mode); 
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        current_execution_status_ = ExecutionState::IDLE;   // set to IDLE to allow controller to send new commands
          return;
        } 
        else{ // execute single primitive directly
          RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), 
                "Executing moveC with via_pose: [%f, %f, %f, %f, %f, %f], goal_pose: [%f, %f, %f, %f, %f, %f], velocity: %f, acceleration: %f, blend_radius: %f, mode: %d", 
                via_pose.x, via_pose.y, via_pose.z, via_pose.rx, via_pose.ry, via_pose.rz,
                goal_pose.x, goal_pose.y, goal_pose.z, goal_pose.rx, goal_pose.ry, goal_pose.rz,
                velocity, acceleration, blend_radius, mode);          
          bool success = instruction_executor_->moveC(via_pose, goal_pose, acceleration, velocity, blend_radius, mode);
          current_execution_status_ = success ? ExecutionState::SUCCESS : ExecutionState::ERROR;
          RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), " [processMotionCommand] After executing moveC: current_execution_status_ = %d", current_execution_status_.load());return;
        }
        break;
      }

      case 33: { 
        // TODO(mathias31415): remove case 33 block with hardcoded motion sequence, only for testing
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Sending hardcoded motion sequence");
        std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> motion_sequence{
          std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.57, 0, 0, 0, 0 }, 0.1, std::chrono::seconds(0), 1.0, 1.0),
          std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }, 0.1, std::chrono::seconds(2)),
          std::make_shared<urcl::control::MoveLPrimitive>(urcl::Pose(-0.203, 0.263, 0.559, 0.68, -1.083, -2.076), 0.1, std::chrono::seconds(0), 0.5, 2.0),
          std::make_shared<urcl::control::MoveLPrimitive>(urcl::Pose{ -0.203, 0.463, 0.559, 0.68, -1.083, -2.076 }, 0.1, std::chrono::seconds(2)),
          std::make_shared<urcl::control::MoveCPrimitive>(
            urcl::Pose(-0.150, 0.350, 0.550, 0.68, -1.0, -2.0),  // via (Zwischenpose)
            urcl::Pose(-0.100, 0.400, 0.550, 0.68, -1.0, -2.0),  // target (Zielpose)
            0.05,  // acceleration
            0.2,  // velocity
            0.0, // blend_radius
            0),    // mode
          std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.57, 0, 0, 0, 0 }, 0.1, std::chrono::seconds(0), 1.0, 1.0),
          std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }, 0.1, std::chrono::seconds(2)),
          std::make_shared<urcl::control::MoveLPrimitive>(urcl::Pose(-0.203, 0.263, 0.559, 0.68, -1.083, -2.076), 0.1, std::chrono::seconds(0), 0.5, 2.0),
          std::make_shared<urcl::control::MoveLPrimitive>(urcl::Pose{ -0.203, 0.463, 0.559, 0.68, -1.083, -2.076 }, 0.1, std::chrono::seconds(2)),
          std::make_shared<urcl::control::MoveCPrimitive>(
            urcl::Pose(-0.150, 0.350, 0.550, 0.68, -1.0, -2.0),  // via (Zwischenpose)
            urcl::Pose(-0.100, 0.400, 0.550, 0.68, -1.0, -2.0),  // target (Zielpose)
            0.05,  // acceleration
            0.2,  // velocity
            0.0, // blend_radius
            0)  
        };
        bool success = instruction_executor_->executeMotion(motion_sequence);
        current_execution_status_ = success ? ExecutionState::SUCCESS : ExecutionState::ERROR;
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), " [processMotionCommand] After executing motion sequence: current_execution_status_ = %d", current_execution_status_.load());
        break;
      }
      
      default: {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Invalid motion command: motion type %f is not supported", motion_type);
        current_execution_status_ = ExecutionState::ERROR;
        return;
      }
    }

  } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Failed to execute motion command: %s", e.what());
      current_execution_status_ = ExecutionState::ERROR;
  }
}

void MotionPrimitivesUrDriver::handleRobotProgramState(bool program_running)
{
  robot_program_running_ = program_running;
}

// TODO (mathias31415): check if this is correct with known values (XYZ-Order?)
// Convert quaternion to Euler angles (roll, pitch, yaw) 
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void MotionPrimitivesUrDriver::quaternionToEuler(double qx, double qy, double qz, double qw, double& rx, double& ry, double& rz) {
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (qw * qx + qy * qz);
  double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  rx = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = std::sqrt(1 + 2 * (qw * qy - qx * qz));
  double cosp = std::sqrt(1 - 2 * (qw * qy - qx * qz));
  ry = 2 * std::atan2(sinp, cosp) - M_PI / 2;

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (qw * qz + qx * qy);
  double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  rz = std::atan2(siny_cosp, cosy_cosp);

  // RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesUrDriver"), "Converted quaternion [%f, %f, %f, %f] to Euler angles: [%f, %f, %f]",qx, qy, qz, qw, rx, ry, rz);
}

bool MotionPrimitivesUrDriver::getMovetimeOrVelocityAndAcceleration(const std::vector<double>& command, double& velocity, double& acceleration, double& move_time) {
  // Check if move_time is valid
  if (!std::isnan(command[24]) && command[24] > 0.0) {
    move_time = command[24];
    velocity = 1.0;        // If move_time is valid, velocity and acceleration are ignored in moveJ and moveL, but must be > 0.0
    acceleration = 1.0;   
    return true;
  }
  // If no valid move_time, check if velocity and acceleration are valid
  else if (!std::isnan(command[22]) && command[22] > 0.0 &&
           !std::isnan(command[23]) && command[23] > 0.0) {
    velocity = command[22];
    acceleration = command[23];
    move_time = 0.0;
    return true;
  }
  // Invalid values
  else {
    RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "move_time, velocity and acceleration are all invalid");
    return false;
  }
}

bool MotionPrimitivesUrDriver::getVelocityAndAcceleration(const std::vector<double>& command, double& velocity, double& acceleration, double& move_time) {
  // Check if velocity and acceleration are valid
  if (!std::isnan(command[22]) && command[22] > 0.0 &&
           !std::isnan(command[23]) && command[23] > 0.0) {
    velocity = command[22];
    acceleration = command[23];
    move_time = 0.0;
    return true;
  }
  // Invalid values
  else {
    RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesUrDriver"), "velocity or acceleration is invalid");
    return false;
  }
}

}  // namespace ur_robot_motion_primitives_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ur_robot_motion_primitives_driver::MotionPrimitivesUrDriver, hardware_interface::SystemInterface)