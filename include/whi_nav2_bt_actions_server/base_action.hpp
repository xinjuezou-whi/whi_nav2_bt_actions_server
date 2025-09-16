/******************************************************************
abstract base class for bt action plugins

Features:
- virtual interface
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-09-08: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/action/spin_to_path.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <nav2_costmap_2d/costmap_topic_collision_checker.hpp>
#include <nav2_util/simple_action_server.hpp>

#include <string>

namespace whi_nav2_bt_actions_server
{
    enum class CostmapInfoType
    {
        NONE = 0,
        LOCAL = 1,
        GLOBAL = 2,
        BOTH = 3
    };

    class BaseAction
    {
    public:
        using Ptr = std::shared_ptr<BaseAction>;

        BaseAction() = default;
        virtual ~BaseAction() = default;

    public:
        virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr Parent,
            const std::string& Name, std::shared_ptr<tf2_ros::Buffer> Tf,
            std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> LocalCollisionChecker,
            std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> GlobalCollisionChecker) = 0;
        virtual void cleanup() = 0;
        virtual void activate() = 0;
        virtual void deactivate() = 0;
        virtual CostmapInfoType getResourceInfo() = 0;
    };

    enum class Status : int8_t
    {
        SUCCEEDED = 1,
        FAILED = 2,
        RUNNING = 3,
    };

    struct ResultStatus
    {
        Status status;
        uint16_t error_code{0};
        std::string error_msg;
    };

    template<typename ActionT>
    class BaseActionT : public BaseAction
    {
    public:
        using ActionServer = nav2_util::SimpleActionServer<ActionT>;

        BaseActionT()
            : action_server_(nullptr), cycle_frequency_(10.0), enabled_(false) {};
        virtual ~BaseActionT() = default;

    public:
        // Derived classes can override this method to catch the command and perform some checks
        // before getting into the main loop. The method will only be called
        // once and should return SUCCEEDED otherwise behavior will return FAILED.
        virtual ResultStatus onRun(const std::shared_ptr<const typename ActionT::Goal> Command) = 0;

        // This is the method derived classes should mainly implement
        // and will be called cyclically while it returns RUNNING.
        // Implement the behavior such that it runs some unit of work on each call
        // and provides a status. The recovery will finish once SUCCEEDED is returned
        // It's up to the derived class to define the final commanded velocity.
        virtual ResultStatus onCycleUpdate() = 0;

        // an opportunity for derived classes to do something on configuration
        // if they chose
        virtual void onConfigure() {}

        // an opportunity for derived classes to do something on cleanup
        // if they chose
        virtual void onCleanup() {}

        // an opportunity for a derived class to do something on action completion
        virtual void onActionCompletion(std::shared_ptr<typename ActionT::Result>/*result*/) {}

        void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr Parent,
            const std::string& Name, std::shared_ptr<tf2_ros::Buffer> Tf,
            std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> LocalCollisionChecker,
            std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> GlobalCollisionChecker) override
        {
            RCLCPP_INFO(Parent->get_logger(), "Configuring %s", Name.c_str());

            node_ = Parent;
            auto node = node_.lock();
            logger_ = node->get_logger();
            clock_ = node->get_clock();

            RCLCPP_INFO(logger_, "Configuring %s", Name.c_str());

            action_name_ = Name;
            tf_ = Tf;

            node->get_parameter("cycle_frequency", cycle_frequency_);
            node->get_parameter("local_frame", local_frame_);
            node->get_parameter("global_frame", global_frame_);
            node->get_parameter("robot_base_frame", robot_base_frame_);
            node->get_parameter("transform_tolerance", transform_tolerance_);

            if (!node->has_parameter("action_server_result_timeout"))
            {
                node->declare_parameter("action_server_result_timeout", 10.0);
            }

            double actionServerResultTimeout;
            node->get_parameter("action_server_result_timeout", actionServerResultTimeout);
            rcl_action_server_options_t serverOptions = rcl_action_server_get_default_options();
            serverOptions.result_timeout.nanoseconds = RCL_S_TO_NS(actionServerResultTimeout);

            action_server_ = std::make_shared<ActionServer>(node, action_name_,
                std::bind(&BaseActionT::execute, this), nullptr,
                std::chrono::milliseconds(500), false, serverOptions);

            local_collision_checker_ = LocalCollisionChecker;
            global_collision_checker_ = GlobalCollisionChecker;

            bool useStamped;
            node->get_parameter("use_stamped_vel", useStamped);
            if (useStamped)
            {
                vel_pub_ = node->create_publisher<Twist>("cmd_vel", 1);
            }
            else
            {
                vel_unstamped_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
            }

            onConfigure();
        }

        void cleanup() override
        {
            action_server_.reset();
            vel_pub_.reset();
            vel_unstamped_pub_.reset();
            
            onCleanup();
        }

        void activate() override
        {
            RCLCPP_INFO(logger_, "Activating %s", action_name_.c_str());

            if (vel_pub_)
            {
                vel_pub_->on_activate();
            }
            else
            {
                vel_unstamped_pub_->on_activate();
            }
            enabled_ = true;
        }

        void deactivate() override
        {
            if (vel_pub_)
            {
                vel_pub_->on_deactivate();
            }
            else
            {
                vel_unstamped_pub_->on_deactivate();
            }
            action_server_->deactivate();
            enabled_ = false;
        }

    protected:
        using Twist = geometry_msgs::msg::TwistStamped;
        void execute()
        {
            RCLCPP_INFO(logger_, "Attempting %s", action_name_.c_str());

            if (!enabled_)
            {
                RCLCPP_WARN(logger_, "Called while inactive, ignoring request.");
                return;
            }

            // Initialize the ActionT result
            auto result = std::make_shared<typename ActionT::Result>();

            ResultStatus onRunResult = onRun(action_server_->get_current_goal());
            if (onRunResult.status != Status::SUCCEEDED)
            {
                result->error_code = onRunResult.error_code;
                result->error_msg = onRunResult.error_msg;
                RCLCPP_INFO(logger_, "Initial checks failed for %s - %s", action_name_.c_str(),
                    onRunResult.error_msg.c_str());
                action_server_->terminate_current(result);
                return;
            }

            auto startTime = clock_->now();
            rclcpp::Rate loop_rate(cycle_frequency_);

            while (rclcpp::ok())
            {
                elapsed_time_ = clock_->now() - startTime;

                if (action_server_->is_preempt_requested())
                {
                    RCLCPP_ERROR(logger_, "Received a preemption request for %s,"
                        " however feature is currently not implemented. Aborting and stopping.",
                        action_name_.c_str());
                    stopRobot();
                    result->total_elapsed_time = clock_->now() - startTime;
                    onActionCompletion(result);
                    action_server_->terminate_current(result);

                    return;
                }

                if (action_server_->is_cancel_requested())
                {
                    RCLCPP_INFO(logger_, "Canceling %s", action_name_.c_str());
                    stopRobot();
                    result->total_elapsed_time = elapsed_time_;
                    onActionCompletion(result);
                    action_server_->terminate_all(result);

                    return;
                }


                ResultStatus onCycleUpdateResult = onCycleUpdate();
                switch (onCycleUpdateResult.status)
                {
                case Status::SUCCEEDED:
                    RCLCPP_INFO(logger_, "%s completed successfully", action_name_.c_str());
                    result->total_elapsed_time = clock_->now() - startTime;
                    onActionCompletion(result);
                    action_server_->succeeded_current(result);
                    return;
                case Status::FAILED:
                    result->error_code = onCycleUpdateResult.error_code;
                    result->error_msg = action_name_ + " failed:" + onCycleUpdateResult.error_msg;
                    RCLCPP_WARN(logger_, result->error_msg.c_str());
                    result->total_elapsed_time = clock_->now() - startTime;
                    onActionCompletion(result);
                    action_server_->terminate_current(result);
                    return;
                case Status::RUNNING:
                default:
                    loop_rate.sleep();
                    break;
                }
            }
        }

        void stopRobot()
        {
            Twist msgTwist;
            msgTwist.header.stamp = clock_->now();
            msgTwist.twist.linear.x = 0.0;
            msgTwist.twist.linear.y = 0.0;
            msgTwist.twist.angular.z = 0.0;
            if (vel_pub_)
            {
                vel_pub_->publish(msgTwist);
            }
            else
            {
                vel_unstamped_pub_->publish(msgTwist.twist);
            }
        }

    protected:
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

        std::string action_name_;
        rclcpp_lifecycle::LifecyclePublisher<Twist>::SharedPtr vel_pub_{ nullptr };
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_unstamped_pub_{ nullptr };
        std::shared_ptr<ActionServer> action_server_{ nullptr };
        std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> local_collision_checker_{ nullptr };
        std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> global_collision_checker_{ nullptr };
        std::shared_ptr<tf2_ros::Buffer> tf_{ nullptr };

        double cycle_frequency_;
        double enabled_;
        std::string local_frame_{ "odom" };
        std::string global_frame_{ "map" };
        std::string robot_base_frame_{ "base_link" };
        double transform_tolerance_{ 0.25 };

        rclcpp::Duration elapsed_time_{0, 0};

        // Clock
        rclcpp::Clock::SharedPtr clock_;

        // Logger
        rclcpp::Logger logger_{ rclcpp::get_logger("whi_nav2_bt_actions_server") };
    };
} // namespace whi_pose_registration_server
