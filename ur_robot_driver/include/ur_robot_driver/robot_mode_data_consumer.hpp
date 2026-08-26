// Copyright 2026, Arthur Haffemayer
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

  /*!\file
   *
   * \author  Arthur Haffemayer <arthur.haffemayer@gmail.com>
   * \date    2026-08-03
   *
   */

#ifndef UR_ROBOT_DRIVER__ROBOT_MODE_DATA_CONSUMER_HPP_
#define UR_ROBOT_DRIVER__ROBOT_MODE_DATA_CONSUMER_HPP_

#include <atomic>
#include <cstdint>

#include "ur_client_library/primary/primary_consumer.h"
#include "ur_client_library/primary/robot_state/robot_mode_data.h"
#include "ur_msgs/msg/robot_mode_data_msg.hpp"

namespace ur_robot_driver
{
/*!
 * \brief Receives the primary interface's RobotModeData into a lock-free buffer.
 *
 * Registered against the PrimaryClient so that its receive thread writes here directly.
 */
class RobotModeDataConsumer : public urcl::primary_interface::PrimaryConsumer
{
public:
  bool consume(urcl::primary_interface::RobotModeData& pkg) override
  {
    control_mode_.store(pkg.control_mode_, std::memory_order_relaxed);
    return true;
  }
  uint8_t control_mode() const
  {
    return control_mode_.load(std::memory_order_relaxed);
  }

private:
  std::atomic<uint8_t> control_mode_{ ur_msgs::msg::RobotModeDataMsg::CONTROL_MODE_UNKNOWN };
};
}  // namespace ur_robot_driver
#endif  // UR_ROBOT_DRIVER__ROBOT_MODE_DATA_CONSUMER_HPP_
