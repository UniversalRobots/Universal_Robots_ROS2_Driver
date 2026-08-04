// Copyright 2026 Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "ur_robot_driver/mode_compatibility.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ur_robot_driver
{
namespace
{
constexpr const char* PASSTHROUGH_GPIO = "trajectory_passthrough";
constexpr const char* FORCE_MODE_GPIO = "force_mode";
constexpr const char* FREEDRIVE_MODE_GPIO = "freedrive_mode";
constexpr const char* TOOL_CONTACT_GPIO = "tool_contact";
constexpr const char* TWIST_GPIO = "twist";
constexpr const char* HW_IF_MOTION_PRIMITIVES = "motion_primitive";
}  // namespace

ModeCompatibilityMatrix createModeCompatibilityMatrix()
{
  ModeCompatibilityMatrix mode_compatibility;

  mode_compatibility[hardware_interface::HW_IF_POSITION][hardware_interface::HW_IF_VELOCITY] = false;
  mode_compatibility[hardware_interface::HW_IF_POSITION][hardware_interface::HW_IF_EFFORT] = false;
  mode_compatibility[hardware_interface::HW_IF_POSITION][FORCE_MODE_GPIO] = false;
  mode_compatibility[hardware_interface::HW_IF_POSITION][PASSTHROUGH_GPIO] = false;
  mode_compatibility[hardware_interface::HW_IF_POSITION][FREEDRIVE_MODE_GPIO] = false;
  mode_compatibility[hardware_interface::HW_IF_POSITION][TOOL_CONTACT_GPIO] = true;
  mode_compatibility[hardware_interface::HW_IF_POSITION][HW_IF_MOTION_PRIMITIVES] = false;
  mode_compatibility[hardware_interface::HW_IF_POSITION][TWIST_GPIO] = false;

  mode_compatibility[hardware_interface::HW_IF_VELOCITY][hardware_interface::HW_IF_POSITION] = false;
  mode_compatibility[hardware_interface::HW_IF_VELOCITY][hardware_interface::HW_IF_EFFORT] = false;
  mode_compatibility[hardware_interface::HW_IF_VELOCITY][FORCE_MODE_GPIO] = false;
  mode_compatibility[hardware_interface::HW_IF_VELOCITY][PASSTHROUGH_GPIO] = false;
  mode_compatibility[hardware_interface::HW_IF_VELOCITY][FREEDRIVE_MODE_GPIO] = false;
  mode_compatibility[hardware_interface::HW_IF_VELOCITY][TOOL_CONTACT_GPIO] = true;
  mode_compatibility[hardware_interface::HW_IF_VELOCITY][HW_IF_MOTION_PRIMITIVES] = false;
  mode_compatibility[hardware_interface::HW_IF_VELOCITY][TWIST_GPIO] = false;

  mode_compatibility[hardware_interface::HW_IF_EFFORT][hardware_interface::HW_IF_POSITION] = false;
  mode_compatibility[hardware_interface::HW_IF_EFFORT][hardware_interface::HW_IF_VELOCITY] = false;
  mode_compatibility[hardware_interface::HW_IF_EFFORT][FORCE_MODE_GPIO] = false;
  mode_compatibility[hardware_interface::HW_IF_EFFORT][PASSTHROUGH_GPIO] = false;
  mode_compatibility[hardware_interface::HW_IF_EFFORT][FREEDRIVE_MODE_GPIO] = false;
  mode_compatibility[hardware_interface::HW_IF_EFFORT][TOOL_CONTACT_GPIO] = true;
  mode_compatibility[hardware_interface::HW_IF_EFFORT][HW_IF_MOTION_PRIMITIVES] = false;

  mode_compatibility[FORCE_MODE_GPIO][hardware_interface::HW_IF_POSITION] = false;
  mode_compatibility[FORCE_MODE_GPIO][hardware_interface::HW_IF_VELOCITY] = false;
  mode_compatibility[FORCE_MODE_GPIO][hardware_interface::HW_IF_EFFORT] = false;
  mode_compatibility[FORCE_MODE_GPIO][PASSTHROUGH_GPIO] = true;
  mode_compatibility[FORCE_MODE_GPIO][FREEDRIVE_MODE_GPIO] = false;
  mode_compatibility[FORCE_MODE_GPIO][TOOL_CONTACT_GPIO] = false;
  mode_compatibility[FORCE_MODE_GPIO][HW_IF_MOTION_PRIMITIVES] = true;
  mode_compatibility[FORCE_MODE_GPIO][TWIST_GPIO] = true;

  mode_compatibility[PASSTHROUGH_GPIO][hardware_interface::HW_IF_POSITION] = false;
  mode_compatibility[PASSTHROUGH_GPIO][hardware_interface::HW_IF_VELOCITY] = false;
  mode_compatibility[PASSTHROUGH_GPIO][hardware_interface::HW_IF_EFFORT] = false;
  mode_compatibility[PASSTHROUGH_GPIO][FORCE_MODE_GPIO] = true;
  mode_compatibility[PASSTHROUGH_GPIO][FREEDRIVE_MODE_GPIO] = false;
  mode_compatibility[PASSTHROUGH_GPIO][TOOL_CONTACT_GPIO] = true;
  mode_compatibility[PASSTHROUGH_GPIO][HW_IF_MOTION_PRIMITIVES] = false;
  mode_compatibility[PASSTHROUGH_GPIO][TWIST_GPIO] = false;

  mode_compatibility[FREEDRIVE_MODE_GPIO][hardware_interface::HW_IF_POSITION] = false;
  mode_compatibility[FREEDRIVE_MODE_GPIO][hardware_interface::HW_IF_VELOCITY] = false;
  mode_compatibility[FREEDRIVE_MODE_GPIO][hardware_interface::HW_IF_EFFORT] = false;
  mode_compatibility[FREEDRIVE_MODE_GPIO][FORCE_MODE_GPIO] = false;
  mode_compatibility[FREEDRIVE_MODE_GPIO][PASSTHROUGH_GPIO] = false;
  mode_compatibility[FREEDRIVE_MODE_GPIO][TOOL_CONTACT_GPIO] = false;
  mode_compatibility[FREEDRIVE_MODE_GPIO][HW_IF_MOTION_PRIMITIVES] = false;
  mode_compatibility[FREEDRIVE_MODE_GPIO][TWIST_GPIO] = false;

  mode_compatibility[TOOL_CONTACT_GPIO][hardware_interface::HW_IF_POSITION] = true;
  mode_compatibility[TOOL_CONTACT_GPIO][hardware_interface::HW_IF_VELOCITY] = true;
  mode_compatibility[TOOL_CONTACT_GPIO][hardware_interface::HW_IF_EFFORT] = true;
  mode_compatibility[TOOL_CONTACT_GPIO][FORCE_MODE_GPIO] = false;
  mode_compatibility[TOOL_CONTACT_GPIO][PASSTHROUGH_GPIO] = true;
  mode_compatibility[TOOL_CONTACT_GPIO][FREEDRIVE_MODE_GPIO] = false;
  mode_compatibility[TOOL_CONTACT_GPIO][HW_IF_MOTION_PRIMITIVES] = true;
  mode_compatibility[TOOL_CONTACT_GPIO][TWIST_GPIO] = true;

  mode_compatibility[HW_IF_MOTION_PRIMITIVES][hardware_interface::HW_IF_POSITION] = false;
  mode_compatibility[HW_IF_MOTION_PRIMITIVES][hardware_interface::HW_IF_VELOCITY] = false;
  mode_compatibility[HW_IF_MOTION_PRIMITIVES][hardware_interface::HW_IF_EFFORT] = false;
  mode_compatibility[HW_IF_MOTION_PRIMITIVES][FORCE_MODE_GPIO] = true;
  mode_compatibility[HW_IF_MOTION_PRIMITIVES][PASSTHROUGH_GPIO] = false;
  mode_compatibility[HW_IF_MOTION_PRIMITIVES][FREEDRIVE_MODE_GPIO] = false;
  mode_compatibility[HW_IF_MOTION_PRIMITIVES][TOOL_CONTACT_GPIO] = true;
  mode_compatibility[HW_IF_MOTION_PRIMITIVES][TWIST_GPIO] = false;

  mode_compatibility[TWIST_GPIO][hardware_interface::HW_IF_POSITION] = false;
  mode_compatibility[TWIST_GPIO][hardware_interface::HW_IF_VELOCITY] = false;
  mode_compatibility[TWIST_GPIO][FORCE_MODE_GPIO] = true;
  mode_compatibility[TWIST_GPIO][PASSTHROUGH_GPIO] = false;
  mode_compatibility[TWIST_GPIO][FREEDRIVE_MODE_GPIO] = false;
  mode_compatibility[TWIST_GPIO][TOOL_CONTACT_GPIO] = true;
  mode_compatibility[TWIST_GPIO][HW_IF_MOTION_PRIMITIVES] = false;

  return mode_compatibility;
}

}  // namespace ur_robot_driver
