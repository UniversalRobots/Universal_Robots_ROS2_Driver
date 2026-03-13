#pragma once

#include "rclcpp/rclcpp.hpp"
#include "ur_rtde_client/rtde_manager.hpp"
#include "ur_rtde_client/publisher.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ur_rtde_client/publisher.hpp"

namespace ur_rtde_client
{
    /**
     * @brief ROS 2 node that bridges UR RTDE data to ROS topics.
     */
    class RtdeClientNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructs the node, declares parameters, loads config, creates publishers,
         *        and starts RTDE communication (which runs in a background thread).
         */
        RtdeClientNode();

    private:
        /// @brief Helper that loads YAML mapping and creates ROS publishers per RTDE variable/group.
        std::unique_ptr<Publisher> publisher_;

        /// @brief Manager that owns the URCL RTDEClient, initializes and streams data.
        std::unique_ptr<RtdeManager> rtde_manager_;

        /// @brief Recipe (list of RTDE output keys) requested to the robot and published by this node.
        std::vector<std::string> output_recipe_;

        /// @brief Robot controller IP address (declared/read as parameter).
        std::string robot_ip_;

        /// @brief Keys actually created and active after validating with the YAML mapping.
        std::vector<std::string> effective_keys_;
    };

} // namespace ur_rtde_client