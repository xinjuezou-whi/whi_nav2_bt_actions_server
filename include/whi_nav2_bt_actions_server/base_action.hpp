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

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <nav2_costmap_2d/costmap_topic_collision_checker.hpp>
#include <nav2_util/simple_action_server.hpp>

#include <string>

namespace whi_nav2_bt_actions_server
{
    class BaseAction
    {
    public:
        using Ptr = std::shared_ptr<BaseAction>;

        BaseAction() = default;
        virtual ~BaseAction() = default;

    public:
        virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
            const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) = 0;
        virtual void cleanup() = 0;
        virtual void activate() = 0;
        virtual void deactivate() = 0;
    };

    enum class Status : int8_t
    {
        SUCCEEDED = 1,
        FAILED = 2,
        RUNNING = 3,
    };

    template<typename ActionT>
    class BaseActionT : public BaseAction
    {
    public:
        using ActionServer = nav2_util::SimpleActionServer<ActionT, rclcpp_lifecycle::LifecycleNode>;

        BaseActionT()
            : action_server_(nullptr), cycle_frequency_(10.0), enabled_(false) {};
        virtual ~BaseActionT() = default;

    public:
        // Derived classes can override this method to catch the command and perform some checks
        // before getting into the main loop. The method will only be called
        // once and should return SUCCEEDED otherwise behavior will return FAILED.
        virtual Status onRun(const std::shared_ptr<const typename ActionT::Goal> Command) = 0;

        // This is the method derived classes should mainly implement
        // and will be called cyclically while it returns RUNNING.
        // Implement the behavior such that it runs some unit of work on each call
        // and provides a status. The recovery will finish once SUCCEEDED is returned
        // It's up to the derived class to define the final commanded velocity.
        virtual Status onCycleUpdate() = 0;

        // an opportunity for derived classes to do something on configuration
        // if they chose
        virtual void onConfigure() {}

        // an opportunity for derived classes to do something on cleanup
        // if they chose
        virtual void onCleanup() {}

        void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
            const std::string& name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker)
        {
            RCLCPP_INFO(parent->get_logger(), "Configuring %s", name.c_str());

            node_ = parent;
            action_name_ = name;
            tf_ = tf;

            node_->get_parameter("cycle_frequency", cycle_frequency_);
            node_->get_parameter("global_frame", global_frame_);
            node_->get_parameter("robot_base_frame", robot_base_frame_);
            node_->get_parameter("transform_tolerance", transform_tolerance_);

            action_server_ = std::make_shared<ActionServer>(node_, action_name_,
                std::bind(&BaseActionT::execute, this));

            collision_checker_ = collision_checker;

            vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

            onConfigure();
        }

        void cleanup()
        {
            action_server_.reset();
            vel_pub_.reset();
            
            onCleanup();
        }

        void activate()
        {
            RCLCPP_INFO(node_->get_logger(), "Activating %s", action_name_.c_str());

            vel_pub_->on_activate();
            enabled_ = true;
        }

        void deactivate()
        {
            vel_pub_->on_deactivate();
            enabled_ = false;
        }

    protected:
        void execute()
        {
            RCLCPP_INFO(node_->get_logger(), "Attempting %s", action_name_.c_str());

            if (!enabled_)
            {
                RCLCPP_WARN(node_->get_logger(), "Called while inactive, ignoring request.");
                return;
            }

            if (onRun(action_server_->get_current_goal()) != Status::SUCCEEDED)
            {
                RCLCPP_INFO(node_->get_logger(), "Initial checks failed for %s", action_name_.c_str());
                action_server_->terminate_current();

                return;
            }

            // Log a message every second
            auto timer = node_->create_wall_timer(std::chrono::seconds(1), [&]()
                { RCLCPP_INFO(node_->get_logger(), "%s running...", action_name_.c_str()); });

            auto start_time = steady_clock_.now();

            // Initialize the ActionT result
            auto result = std::make_shared<typename ActionT::Result>();

            rclcpp::Rate loop_rate(cycle_frequency_);

            while (rclcpp::ok())
            {
                if (action_server_->is_cancel_requested())
                {
                    RCLCPP_INFO(node_->get_logger(), "Canceling %s", action_name_.c_str());
                    stopRobot();
                    result->total_elapsed_time = steady_clock_.now() - start_time;
                    action_server_->terminate_all(result);
                    return;
                }

                if (action_server_->is_preempt_requested())
                {
                    RCLCPP_ERROR(node_->get_logger(), "Received a preemption request for %s,"
                        " however feature is currently not implemented. Aborting and stopping.",
                        action_name_.c_str());
                    stopRobot();
                    result->total_elapsed_time = steady_clock_.now() - start_time;
                    action_server_->terminate_current(result);

                    return;
                }

                switch (onCycleUpdate())
                {
                case Status::SUCCEEDED:
                    RCLCPP_INFO(node_->get_logger(), "%s completed successfully", action_name_.c_str());
                    result->total_elapsed_time = steady_clock_.now() - start_time;
                    action_server_->succeeded_current(result);
                    return;
                case Status::FAILED:
                    RCLCPP_WARN(node_->get_logger(), "%s failed", action_name_.c_str());
                    result->total_elapsed_time = steady_clock_.now() - start_time;
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
            auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
            cmd_vel->linear.x = 0.0;
            cmd_vel->linear.y = 0.0;
            cmd_vel->angular.z = 0.0;

            vel_pub_->publish(std::move(cmd_vel));
        }

    protected:
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        std::string action_name_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        std::shared_ptr<ActionServer> action_server_;
        std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
        std::shared_ptr<tf2_ros::Buffer> tf_;

        double cycle_frequency_;
        double enabled_;
        std::string global_frame_;
        std::string robot_base_frame_;
        double transform_tolerance_;

        // Clock
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    };
} // namespace whi_pose_registration_server
