// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
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
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-09-11
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/comm/shell_consumer.h>
#include <ur_client_library/primary/primary_parser.h>

#ifdef ROS_BUILD
#include <console_bridge/console.h>
#endif

using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a better configuration
// system such as Boost.Program_options
const std::string ROBOT_IP = "192.168.56.101";

int main(int argc, char* argv[])
{
#ifdef ROS_BUILD
  // When compiled with ROS support we have to set the logging level in order to see output with a
  // lower level than WARNING
  console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
#endif

  // First of all, we need a stream that connects to the robot
  comm::URStream<primary_interface::PrimaryPackage> primary_stream(ROBOT_IP, urcl::primary_interface::UR_PRIMARY_PORT);

  // This will parse the primary packages
  primary_interface::PrimaryParser parser;

  // The producer needs both, the stream and the parser to fully work
  comm::URProducer<primary_interface::PrimaryPackage> prod(primary_stream, parser);
  prod.setupProducer();

  // The shell consumer will print the package contents to the shell
  std::unique_ptr<comm::IConsumer<primary_interface::PrimaryPackage>> consumer;
  consumer.reset(new comm::ShellConsumer<primary_interface::PrimaryPackage>());

  // The notifer will be called at some points during connection setup / loss. This isn't fully
  // implemented atm.
  comm::INotifier notifier;

  // Now that we have all components, we can create and start the pipeline to run it all.
  comm::Pipeline<primary_interface::PrimaryPackage> pipeline(prod, consumer.get(), "Pipeline", notifier);
  pipeline.run();

  // Package contents will be printed while not being interrupted
  // Note: Packages for which the parsing isn't implemented, will only get their raw bytes printed.
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}
