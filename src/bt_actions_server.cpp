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
#include "whi_nav2_bt_actions_server/bt_actions_server.hpp"

#include <tf2_ros/create_timer_ros.h>
#include <nav2_util/node_utils.hpp>

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>

namespace whi_nav2_bt_actions_server
{
	BtActionsServer::BtActionsServer()
		: LifecycleNode("whi_nav2_bt_actions_server", "", true)
		, plugin_loader_("whi_nav2_bt_actions_server", "whi_nav2_bt_actions_server::BaseAction")
		, default_ids_{"SpinToPath"}
		, default_types_{"whi_nav2_bt_actions_server/SpinToPath"}
	{
		declare_parameter("costmap_topic",
			rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
		declare_parameter("footprint_topic",
			rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));
		declare_parameter("cycle_frequency", rclcpp::ParameterValue(10.0));
		declare_parameter("action_plugins", default_ids_);

		declare_parameter("global_frame", rclcpp::ParameterValue(std::string("odom")));
		declare_parameter("robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
		declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.2));

		declare_parameter("use_stamped_vel", rclcpp::ParameterValue(true));
	}

	BtActionsServer::~BtActionsServer()
	{
	}

	nav2_util::CallbackReturn BtActionsServer::on_configure(const rclcpp_lifecycle::State& /*state*/)
	{
		RCLCPP_INFO(get_logger(), "Configuring");

		tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
		auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
			get_node_base_interface(),
			get_node_timers_interface());
		tf_->setCreateTimerInterface(timer_interface);
		transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

		std::string topicCostmap, topicFootprint;
		this->get_parameter("costmap_topic", topicCostmap);
		this->get_parameter("footprint_topic", topicFootprint);
		this->get_parameter("transform_tolerance", transform_tolerance_);
		costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
			shared_from_this(), topicCostmap);
		footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
			shared_from_this(), topicFootprint, 1.0);

		std::string frameGlobal, frameRobotBase;
		get_parameter("global_frame", frameGlobal);
		get_parameter("robot_base_frame", frameRobotBase);
		collision_checker_ = std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
			*costmap_sub_, *footprint_sub_, *tf_, this->get_name(),
			frameGlobal, frameRobotBase, transform_tolerance_);

		get_parameter("action_plugins", action_ids_);
		if (action_ids_ == default_ids_)
		{
			for (size_t i = 0; i < default_ids_.size(); ++i)
			{
				declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
			}
		}
		action_types_.resize(action_ids_.size());
		loadActionPlugins();

		RCLCPP_INFO(get_logger(), "\033[1;32mWHI nav2 bt actions server is configured successfully\033[0m");

		return nav2_util::CallbackReturn::SUCCESS;
	}

	void BtActionsServer::loadActionPlugins()
	{
		auto node = shared_from_this();

		for (size_t i = 0; i != action_ids_.size(); i++)
		{
			action_types_[i] = nav2_util::get_plugin_type_param(node, action_ids_[i]);
			try
			{
				RCLCPP_INFO(get_logger(), "Creating action plugin %s of type %s",
					action_ids_[i].c_str(), action_types_[i].c_str());

				actions_.push_back(plugin_loader_.createUniqueInstance(action_types_[i]));
				actions_.back()->configure(node, action_ids_[i], tf_, collision_checker_);
			}
			catch (const pluginlib::PluginlibException & ex)
			{
				RCLCPP_FATAL(get_logger(), "Failed to create action %s of type %s."
					" Exception: %s", action_ids_[i].c_str(), action_types_[i].c_str(),
					ex.what());
				exit(-1);
			}
		}
	}

	nav2_util::CallbackReturn BtActionsServer::on_activate(const rclcpp_lifecycle::State& /*state*/)
	{
		RCLCPP_INFO(get_logger(), "Activating");

		for (auto& it : actions_)
		{
			it->activate();
		}

		return nav2_util::CallbackReturn::SUCCESS;
	}

	nav2_util::CallbackReturn BtActionsServer::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
	{
		RCLCPP_INFO(get_logger(), "Deactivating");

		for (auto& it : actions_)
		{
			it->deactivate();
		}

		return nav2_util::CallbackReturn::SUCCESS;
	}

	nav2_util::CallbackReturn BtActionsServer::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
	{
		RCLCPP_INFO(get_logger(), "Cleaning up");

		for (auto& it : actions_)
		{
			it->cleanup();
		}

		actions_.clear();
		transform_listener_.reset();
		tf_.reset();
		footprint_sub_.reset();
		costmap_sub_.reset();
		collision_checker_.reset();

		return nav2_util::CallbackReturn::SUCCESS;
	}

	nav2_util::CallbackReturn BtActionsServer::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
	{
		RCLCPP_INFO(get_logger(), "Shutting down");

		return nav2_util::CallbackReturn::SUCCESS;
	}
} // namespace whi_nav2_bt_actions_server
