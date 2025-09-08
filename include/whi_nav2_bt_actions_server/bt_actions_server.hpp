/******************************************************************
pose registration server

Features:
- server frame
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-08-08: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "base_action.hpp"

#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>
#include <vector>

namespace whi_nav2_bt_actions_server
{
	class BtActionsServer : public nav2_util::LifecycleNode
	{
	public:
		BtActionsServer();
		~BtActionsServer();

	public:
		void loadActionPlugins();

	protected:
		nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& State) override;
		nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& State) override;
		nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& State) override;
		nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& State) override;
		nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& State) override;

	protected:
		std::shared_ptr<tf2_ros::Buffer> tf_;
		std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

		// Plugins
		std::vector<pluginlib::UniquePtr<BaseAction>> actions_;
		pluginlib::ClassLoader<BaseAction> plugin_loader_;
		std::vector<std::string> default_ids_;
		std::vector<std::string> default_types_;
		std::vector<std::string> action_ids_;
		std::vector<std::string> action_types_;

		// Utilities
		std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
		std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
		std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

		double transform_tolerance_;
	};

} // namespace whi_nav2_bt_actions_server
