// Copyright 2021 Universal Robots A/S
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

// All source code contained in and/or linked to in this message (the “Source Code”) is subject to the copyright of
// Universal Robots A/S and/or its licensors. THE SOURCE CODE IS PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING – BUT NOT LIMITED TO – WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
// NONINFRINGEMENT. USE OF THE SOURCE CODE IS AT YOUR OWN RISK AND UNIVERSAL ROBOTS A/S AND ITS LICENSORS SHALL, TO THE
// MAXIMUM EXTENT PERMITTED BY LAW, NOT BE LIABLE FOR ANY ERRORS OR MALICIOUS CODE IN THE SOURCE CODE, ANY THIRD-PARTY
// CLAIMS, OR ANY OTHER CLAIMS AND DAMAGES, INCLUDING INDIRECT, INCIDENTAL, SPECIAL, CONSEQUENTIAL OR PUNITIVE DAMAGES,
// OR ANY LOSS OF PROFITS, EXPECTED SAVINGS, OR REVENUES, WHETHER INCURRED DIRECTLY OR INDIRECTLY, OR ANY LOSS OF DATA,
// USE, GOODWILL, OR OTHER INTANGIBLE LOSSES, RESULTING FROM YOUR USE OF THE SOURCE CODE. You may make copies of the
// Source Code for use in connection with a Universal Robots or UR+ product, provided that you include (i) an
// appropriate copyright notice (“©  [the year in which you received the Source Code or the Source Code was first
// published, e.g. “2021”] Universal Robots A/S and/or its licensors”) along with the capitalized section of this notice
// in all copies of the Source Code. By using the Source Code, you agree to the above terms. For more information,
// please contact legal@universal-robots.com.

#include <memory>
#include <utility>

#include "ur_robot_driver/urcl_log_handler.hpp"
#include "rclcpp/logging.hpp"

namespace ur_robot_driver
{
bool g_registered = false;

UrclLogHandler::UrclLogHandler() = default;

void UrclLogHandler::log(const char* file, int line, urcl::LogLevel loglevel, const char* message)
{
  rcutils_log_location_t location = { "", file, static_cast<size_t>(line) };

  const auto logger_name = "UR_Client_Library:" + tf_prefix_;
  switch (loglevel) {
    case urcl::LogLevel::DEBUG:
      rcutils_log(&location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG, logger_name.c_str(), "%s", message);
      break;
    case urcl::LogLevel::INFO:
      rcutils_log(&location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_INFO, logger_name.c_str(), "%s", message);
      break;
    case urcl::LogLevel::WARN:
      rcutils_log(&location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_WARN, logger_name.c_str(), "%s", message);
      break;
    case urcl::LogLevel::ERROR:
      rcutils_log(&location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_ERROR, logger_name.c_str(), "%s", message);
      break;
    case urcl::LogLevel::FATAL:
      rcutils_log(&location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_FATAL, logger_name.c_str(), "%s", message);
      break;
    default:
      break;
  }
}

void registerUrclLogHandler(const std::string& tf_prefix)
{
  if (g_registered == false) {
    std::unique_ptr<UrclLogHandler> log_handler(new UrclLogHandler);
    log_handler->setTFPrefix(tf_prefix);
    // Log level is decided by ROS2 log level
    urcl::setLogLevel(urcl::LogLevel::DEBUG);
    urcl::registerLogHandler(std::move(log_handler));
    g_registered = true;
  }
}

void unregisterUrclLogHandler()
{
  if (g_registered == true) {
    urcl::unregisterLogHandler();
    g_registered = false;
  }
}

}  // namespace ur_robot_driver
