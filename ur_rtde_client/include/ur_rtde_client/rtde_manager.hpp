#pragma once

#include <rclcpp/rclcpp.hpp>
#include "ur_rtde_client/publisher.hpp"
#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/log.h>
#include <ur_client_library/types.h>

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

namespace ur_rtde_client
{
    /**
     * @brief Manages the UR RTDE client lifecycle and bridges data to the Publisher.
     */
    class RtdeManager
    {
    public:
        /**
         * @brief Construct an RtdeManager.
         * @param node Reference to the ROS 2 node.
         * @param robot_ip Robot controller IP address.
         * @param keys RTDE output recipe.
         * @param publisher Reference to the Publisher that will publish parsed data.
         */
        RtdeManager(rclcpp::Node &node,
                    std::string robot_ip,
                    std::vector<std::string> keys,
                    Publisher &publisher);

        /**
         * @brief Destructor of RtdeManager.
         * It reads the last data from the threat before removing the object.
         */
        ~RtdeManager();

        RtdeManager(const RtdeManager &) = delete;
        RtdeManager &operator=(const RtdeManager &) = delete;

        /**
         * @brief Initialize and start the UR RTDE client.
         * @return true if initialization and start succeeded; false otherwise.
         */
        bool initAndStart();

        /**
         * @brief Perform one RTDE cycle, fetch a data package and publish active variables.
         */
        void spinOnce();

    private:
        /// @brief Reference to the ROS 2 node.
        rclcpp::Node &node_;

        /// @brief Robot controller IP address.
        std::string robot_ip_;

        /// @brief RTDE output recipe.
        std::vector<std::string> keys_;

        /// @brief Reference to the Publisher used to publish parsed data.
        Publisher &publisher_;

        /// @brief URCL notifier for logging/callback hooks.
        urcl::comm::INotifier notifier_{};

        /// @brief Owned UR RTDE client.
        std::unique_ptr<urcl::rtde_interface::RTDEClient> rtde_client_;

        /// @brief Reusable buffer holding the latest RTDE data package.
        std::unique_ptr<urcl::rtde_interface::DataPackage> pkg_;

        /// @brief Background thread that reads RTDE data when available.
        std::thread read_thread_;

        /// @brief Flag to signal the read thread to stop.
        std::atomic<bool> stop_reading_{false};

        /// @brief Worker function for the background read thread.
        void readThreadWorker();
    };

    /**
     * @brief Convenience factory that constructs, initializes, and starts an RtdeManager.
     * @param node ROS 2 node reference.
     * @param robot_ip Robot controller IP.
     * @param keys RTDE output recipe.
     * @param publisher Publisher reference.
     * @return A valid 'RtdeManager' on success; 'nullptr' if initialization failed.
     */
    std::unique_ptr<RtdeManager> start_communication(rclcpp::Node &node,
                                                     const std::string &robot_ip,
                                                     const std::vector<std::string> &keys,
                                                     Publisher &publisher);

} // namespace ur_rtde_client