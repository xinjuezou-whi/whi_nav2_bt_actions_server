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

using namespace std::chrono_literals;

namespace whi_nav2_bt_actions_server
{
	BtActionsServer::BtActionsServer()
		: LifecycleNode("recoveries_server", "", true)
		, plugin_loader_("nav2_core", "whi_nav2_bt_actions_server::BaseAction")
		, default_ids_{"spin_to_path"}
		, default_types_{"SpinToPath"}
	{
		declare_parameter("costmap_topic",
			rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
		declare_parameter("footprint_topic",
			rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));
		declare_parameter("cycle_frequency", rclcpp::ParameterValue(10.0));
		declare_parameter("recovery_plugins", default_ids_);

		declare_parameter("global_frame",
			rclcpp::ParameterValue(std::string("odom")));
		declare_parameter("robot_base_frame",
			rclcpp::ParameterValue(std::string("base_link")));
		declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
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

		std::string costmap_topic, footprint_topic;
		this->get_parameter("costmap_topic", costmap_topic);
		this->get_parameter("footprint_topic", footprint_topic);
		this->get_parameter("transform_tolerance", transform_tolerance_);
		costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
			shared_from_this(), costmap_topic);
		footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
			shared_from_this(), footprint_topic, 1.0);

		std::string global_frame, robot_base_frame;
		get_parameter("global_frame", global_frame);
		get_parameter("robot_base_frame", robot_base_frame);
		collision_checker_ = std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
			*costmap_sub_, *footprint_sub_, *tf_, this->get_name(),
			global_frame, robot_base_frame, transform_tolerance_);

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
				RCLCPP_INFO(
					get_logger(), "Creating recovery plugin %s of type %s",
					action_ids_[i].c_str(), action_types_[i].c_str());
				actions_.push_back(plugin_loader_.createUniqueInstance(action_types_[i]));
				actions_.back()->configure(node, action_ids_[i], tf_, collision_checker_);
			}
			catch (const pluginlib::PluginlibException & ex)
			{
				RCLCPP_FATAL(get_logger(), "Failed to create recovery %s of type %s."
					" Exception: %s", action_ids_[i].c_str(), action_types_[i].c_str(),
					ex.what());
				exit(-1);
			}
		}
	}

	nav2_util::CallbackReturn BtActionsServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
	{
		RCLCPP_INFO(get_logger(), "Activating");
		for (auto& it : actions_)
		{
			it->activate();
		}

		return nav2_util::CallbackReturn::SUCCESS;
	}

	nav2_util::CallbackReturn BtActionsServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
	{
		RCLCPP_INFO(get_logger(), "Deactivating");

		for (auto& it : actions_)
		{
			it->deactivate();
		}

		return nav2_util::CallbackReturn::SUCCESS;
	}

	nav2_util::CallbackReturn BtActionsServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
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

	nav2_util::CallbackReturn BtActionsServer::on_shutdown(const rclcpp_lifecycle::State &)
	{
		RCLCPP_INFO(get_logger(), "Shutting down");
		return nav2_util::CallbackReturn::SUCCESS;
	}
} // namespace whi_nav2_bt_actions_server
