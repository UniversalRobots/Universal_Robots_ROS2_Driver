// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-04-09
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_GET_URCONTROL_VERSION_H_INCLUDED
#define UR_CLIENT_LIBRARY_GET_URCONTROL_VERSION_H_INCLUDED

#include "ur_client_library/rtde/rtde_package.h"
#include "ur_client_library/ur/version_information.h"

namespace urcl
{
namespace rtde_interface
{
/*!
 * \brief This class handles the package detailing the UR control version sent by the robot.
 */
class GetUrcontrolVersion : public RTDEPackage
{
public:
  /*!
   * \brief Creates a new GetUrcontrolVersion object.
   */
  GetUrcontrolVersion() : RTDEPackage(PackageType::RTDE_GET_URCONTROL_VERSION)
  {
  }
  virtual ~GetUrcontrolVersion() = default;

  /*!
   * \brief Sets the attributes of the package by parsing a serialized representation of the
   * package.
   *
   * \param bp A parser containing a serialized version of the package
   *
   * \returns True, if the package was parsed successfully, false otherwise
   */
  virtual bool parseWith(comm::BinParser& bp);
  /*!
   * \brief Produces a human readable representation of the package object.
   *
   * \returns A string representing the object
   */
  virtual std::string toString() const;

  VersionInformation version_information_;
};

/*!
 * \brief This class is used to request the used UR control version from the robot.
 */
class GetUrcontrolVersionRequest : public RTDEPackage
{
public:
  /*!
   * \brief Creates a new GetUrcontrolVersionRequest object.
   */
  GetUrcontrolVersionRequest() : RTDEPackage(PackageType::RTDE_GET_URCONTROL_VERSION)
  {
  }
  virtual ~GetUrcontrolVersionRequest() = default;

  /*!
   * \brief Generates a serialized package.
   *
   * \param buffer Buffer to fill with the serialization
   *
   * \returns The total size of the serialized package
   */
  static size_t generateSerializedRequest(uint8_t* buffer);

private:
  static const uint16_t PAYLOAD_SIZE = 0;
};

}  // namespace rtde_interface
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_GET_URCONTROL_VERSION_H_INCLUDED
