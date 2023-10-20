// Copyright 2023, FZI Forschungszentrum Informatik
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner <exner@fzi.de>
 * \date    2023-09-11
 *
 */
//----------------------------------------------------------------------

#pragma once

#include <tf2/tf2/LinearMath/Quaternion.h>

#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <hardware_interface/loaned_state_interface.hpp>
#include <semantic_components/semantic_component_interface.hpp>

namespace ur_controllers
{
class PoseComponent : public semantic_components::SemanticComponentInterface<geometry_msgs::msg::Pose>
{
public:
  explicit PoseComponent(const std::string& name) : SemanticComponentInterface(name, 6)
  {
    interface_names_.emplace_back(name_ + "/" + "position.x");
    interface_names_.emplace_back(name_ + "/" + "position.y");
    interface_names_.emplace_back(name_ + "/" + "position.z");
    interface_names_.emplace_back(name_ + "/" + "orientation.x");
    interface_names_.emplace_back(name_ + "/" + "orientation.y");
    interface_names_.emplace_back(name_ + "/" + "orientation.z");
    interface_names_.emplace_back(name_ + "/" + "orientation.w");

    std::fill(position_.begin(), position_.end(), std::numeric_limits<double>::quiet_NaN());
    std::fill(orientation_.begin(), orientation_.end(), std::numeric_limits<double>::quiet_NaN());
  }

  virtual ~PoseComponent() = default;

  /**
   * Return Pose message with full pose, parent and child frame
   *
   * \return Pose from values;
   */
  bool get_values_as_message(geometry_msgs::msg::Pose& message)
  {
    get_position();
    get_orientation();

    // update the message values
    message.position.x = position_[0];
    message.position.y = position_[1];
    message.position.z = position_[2];

    message.orientation.x = orientation_[0];
    message.orientation.y = orientation_[1];
    message.orientation.z = orientation_[2];
    message.orientation.w = orientation_[3];

    return true;
  }

protected:
  std::array<double, 3> get_position()
  {
    size_t interface_counter = 0;
    for (size_t i = 0; i < 3; ++i) {
      position_[i] = state_interfaces_[interface_counter].get().get_value();
      ++interface_counter;
    }
    return position_;
  }
  std::array<double, 4> get_orientation()
  {
    size_t interface_counter = 3;
    for (size_t i = 0; i < 4; ++i) {
      orientation_[i] = state_interfaces_[interface_counter].get().get_value();
      ++interface_counter;
    }
    return orientation_;
  }
  std::array<double, 3> position_;
  std::array<double, 4> orientation_;
};

}  // namespace ur_controllers
