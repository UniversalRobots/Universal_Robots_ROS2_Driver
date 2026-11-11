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

#ifndef UR_ROBOT_DRIVER__TOOL_CONTACT_REQUEST_HPP_
#define UR_ROBOT_DRIVER__TOOL_CONTACT_REQUEST_HPP_

namespace ur_robot_driver
{
// Values of the tool contact command and state interfaces. They mirror the constants of the same
// names in ur_controllers::ToolContactController.
constexpr double TOOL_CONTACT_STANDBY = 1.0;
constexpr double TOOL_CONTACT_WAITING_BEGIN = 2.0;
constexpr double TOOL_CONTACT_EXECUTING = 3.0;
constexpr double TOOL_CONTACT_FAILURE_BEGIN = 4.0;
constexpr double TOOL_CONTACT_WAITING_END = 5.0;
constexpr double TOOL_CONTACT_SUCCESS_END = 6.0;
constexpr double TOOL_CONTACT_FAILURE_END = 7.0;

enum class ToolContactRequest
{
  NONE,
  START,
  END
};

/*!
 * \brief What the hardware interface has to send to the robot for the tool contact command it reads
 * in one write() cycle.
 *
 * The controller keeps its command at TOOL_CONTACT_WAITING_BEGIN for as long as tool contact is
 * executing, and at TOOL_CONTACT_WAITING_END after a detected contact until the next goal. The
 * command is read in every cycle, but each request must reach the robot only once:
 *
 *  - Starting again would also reset the result to "executing". A result that the robot reported
 *    between the controller's update() and the hardware interface's write() would be overwritten
 *    before the controller could read it, and the robot reports it only once.
 *  - The robot answers every end request it receives outside of a contact with a result of its own.
 *
 * \param commanded_state Value of the tool contact command interface
 * \param current_state Value of the tool contact state interface
 */
inline ToolContactRequest toolContactRequest(const double commanded_state, const double current_state)
{
  if (commanded_state == TOOL_CONTACT_WAITING_BEGIN && current_state != TOOL_CONTACT_EXECUTING) {
    return ToolContactRequest::START;
  }
  if (commanded_state == TOOL_CONTACT_WAITING_END && current_state != TOOL_CONTACT_SUCCESS_END) {
    return ToolContactRequest::END;
  }
  return ToolContactRequest::NONE;
}

}  // namespace ur_robot_driver

#endif  // UR_ROBOT_DRIVER__TOOL_CONTACT_REQUEST_HPP_
