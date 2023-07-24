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

#ifndef UR_ROBOT_DRIVER__URCL_LOG_HANDLER_HPP_
#define UR_ROBOT_DRIVER__URCL_LOG_HANDLER_HPP_

#include <string>
#include "ur_client_library/log.h"
namespace ur_robot_driver
{

/*!
 * \brief Register the UrclLoghHandler, this will start logging messages from the client library with ROS2 logging.
 * This function has to be called inside your node, to enable the log handler.
 */
void registerUrclLogHandler(const std::string& tf_prefix = "");

/*!
 * \brief Unregister the UrclLoghHandler, stop logging messages from the client library with ROS2 logging.
 */
void unregisterUrclLogHandler();

/*!
 * \brief Loghandler for handling messages logged with the C++ client library. This loghandler will log the messages
 * from the client library with ROS2s logging.
 * Use registerLogHandler to register this LogHandler. This class shouldn't be instantiated directly.
 */
class UrclLogHandler : public urcl::LogHandler
{
public:
  /*!
   * \brief Default constructor
   */
  UrclLogHandler();

  /*!
   * \brief Function to log a message
   *
   * \param file The log message comes from this file
   * \param line The log message comes from this line
   * \param loglevel Indicates the severity of the log message
   * \param log Log message
   */
  void log(const char* file, int line, urcl::LogLevel loglevel, const char* message) override;

  /**
   * @brief getTFPrefix - obtain the currently set tf_prefix
   * @return
   */
  const std::string& getTFPrefix() const
  {
    return tf_prefix_;
  }

private:
  std::string tf_prefix_{};

  /**
   * @brief setTFPrefix - set the tf_prefix the logger will append to the node name
   * @param tf_prefix
   */
  void setTFPrefix(const std::string& tf_prefix)
  {
    tf_prefix_ = tf_prefix;
  }

  // Declare the register method as a friend so that we can access setTFPrefix from it
  friend void registerUrclLogHandler(const std::string& tf_prefix);
};

}  // namespace ur_robot_driver

#endif  // UR_ROBOT_DRIVER__URCL_LOG_HANDLER_HPP_
