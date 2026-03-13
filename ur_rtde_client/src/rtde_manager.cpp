#include "ur_rtde_client/rtde_manager.hpp"

using namespace std::chrono_literals;

namespace ur_rtde_client
{
    RtdeManager::RtdeManager(rclcpp::Node &node,
                             std::string robot_ip,
                             std::vector<std::string> keys,
                             Publisher &publisher)
        : node_(node), robot_ip_(std::move(robot_ip)), keys_(std::move(keys)), publisher_(publisher)
    {
    }

    RtdeManager::~RtdeManager()
    {
        stop_reading_ = true;
        if (read_thread_.joinable())
        {
            read_thread_.join();
        }
    }

    bool RtdeManager::initAndStart()
    {
        try
        {
            rtde_client_ = std::make_unique<urcl::rtde_interface::RTDEClient>(
                robot_ip_, notifier_, keys_, std::vector<std::string>{});

            try
            {
                rtde_client_->init();
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_.get_logger(), "RTDE init() UrException for ip=%s: %s",
                             robot_ip_.c_str(), e.what());
                rtde_client_.reset();
                return false;
            }

            rtde_client_->start();

            RCLCPP_INFO(node_.get_logger(),
                        "RTDE client started (ip=%s) with %zu keys.",
                        robot_ip_.c_str(), keys_.size());

            stop_reading_ = false;
            read_thread_ = std::thread(&RtdeManager::readThreadWorker, this);

            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_.get_logger(), "Exception during RTDE init/start: %s", e.what());
            rtde_client_.reset();
            return false;
        }
    }

    void RtdeManager::readThreadWorker()
    {
        RCLCPP_INFO(node_.get_logger(), "RTDE read thread started");

        while (!stop_reading_)
        {
            try
            {
                if (rtde_client_)
                {
                    spinOnce();
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 3000,
                                      "Exception in RTDE read thread: %s", e.what());
            }
        }

        RCLCPP_INFO(node_.get_logger(), "RTDE read thread stopped");
    }

    void RtdeManager::spinOnce()
    {
        if (!rtde_client_)
        {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                                 "RTDE client not initialized. Skipping cycle.");
            return;
        }

        if (!pkg_)
        {
            pkg_ = std::make_unique<urcl::rtde_interface::DataPackage>(keys_);
        }

        auto ok = rtde_client_->getDataPackage(*pkg_, 200ms);
        if (!ok)
        {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                                 "No RTDE package received (timeout).");
            return;
        }

        try
        {
            publisher_.publish(*pkg_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 3000,
                                  "Publisher failed to publish RTDE package: %s", e.what());
        }
    }

    std::unique_ptr<RtdeManager> start_communication(rclcpp::Node &node,
                                                     const std::string &robot_ip,
                                                     const std::vector<std::string> &keys,
                                                     Publisher &publisher)
    {
        auto mgr = std::make_unique<RtdeManager>(node, robot_ip, keys, publisher);
        if (!mgr->initAndStart())
        {
            return nullptr;
        }
        return mgr;
    }

} // namespace ur_rtde_client
