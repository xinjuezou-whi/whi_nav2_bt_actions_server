/******************************************************************
bt action plugin of spin to path

Features:
- spin to align with the planned path
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_nav2_bt_actions_server/plugins/spin_to_path.hpp"

#include <nav2_util/node_utils.hpp>
#include <nav2_util/geometry_utils.hpp>

namespace whi_nav2_bt_actions_server
{
	SpinToPath::SpinToPath()
		: BaseActionT<SpinToPathAction>()
		, feedback_(std::make_shared<SpinToPathAction::Feedback>())
		, prev_yaw_(0.0)
	{
		/// node version and copyright announcement
		std::cout << "\nWHI bt action spin to path VERSION 00.00.1" << std::endl;
		std::cout << "Copyright © 2025-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
	}

	SpinToPath::~SpinToPath() {}

	void SpinToPath::onConfigure()
	{
		nav2_util::declare_parameter_if_not_declared(node_, "simulate_ahead_time", rclcpp::ParameterValue(2.0));
		node_->get_parameter("simulate_ahead_time", simulate_ahead_time_);

		nav2_util::declare_parameter_if_not_declared(node_, "max_rotational_vel", rclcpp::ParameterValue(1.0));
		node_->get_parameter("max_rotational_vel", max_rotational_vel_);

		nav2_util::declare_parameter_if_not_declared(node_, "min_rotational_vel", rclcpp::ParameterValue(0.4));
		node_->get_parameter("min_rotational_vel", min_rotational_vel_);

		nav2_util::declare_parameter_if_not_declared(node_, "rotational_acc_lim", rclcpp::ParameterValue(3.2));
		node_->get_parameter("rotational_acc_lim", rotational_acc_lim_);
	}

	Status SpinToPath::onRun(const std::shared_ptr<const SpinToPathAction::Goal> Command)
	{
		geometry_msgs::msg::PoseStamped currentPose;
		if (!nav2_util::getCurrentPose(currentPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
		{
			RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
			return Status::FAILED;
		}

		prev_yaw_ = tf2::getYaw(currentPose.pose.orientation);
		relative_yaw_ = 0.0;

		auto it = Command->path.poses.begin();
		for (it = Command->path.poses.begin() + 1; it != Command->path.poses.end(); ++it)
		{
			if (nav2_util::geometry_utils::euclidean_distance(Command->path.poses.front().pose, it->pose) >
				Command->lookahead_distance)
			{
				break;
			}
		}
		if (it == Command->path.poses.end())
		{
			it = Command->path.poses.end() - 1;
		}
		cmd_yaw_ = tf2::getYaw(currentPose.pose.orientation) - tf2::getYaw(it->pose.orientation);

		RCLCPP_INFO(node_->get_logger(), "Turning %0.2f for spin recovery.", cmd_yaw_);

		return Status::SUCCEEDED;
	}

	Status SpinToPath::onCycleUpdate()
	{
		geometry_msgs::msg::PoseStamped currentPose;
		if (!nav2_util::getCurrentPose(currentPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
		{
			RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
			return Status::FAILED;
		}

		const double current_yaw = tf2::getYaw(currentPose.pose.orientation);

		double delta_yaw = current_yaw - prev_yaw_;
		if (abs(delta_yaw) > M_PI)
		{
			delta_yaw = copysign(2 * M_PI - abs(delta_yaw), prev_yaw_);
		}

		relative_yaw_ += delta_yaw;
		prev_yaw_ = current_yaw;

		feedback_->angular_distance_traveled = relative_yaw_;
		action_server_->publish_feedback(feedback_);

		double remaining_yaw = abs(cmd_yaw_) - abs(relative_yaw_);
		if (remaining_yaw <= 0)
		{
			stopRobot();
			return Status::SUCCEEDED;
		}

		double vel = sqrt(2 * rotational_acc_lim_ * remaining_yaw);
		vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

		Twist msgTwist;
		msgTwist.header.stamp = steady_clock_.now();
		msgTwist.twist.angular.z = copysign(vel, cmd_yaw_);

		geometry_msgs::msg::Pose2D pose2d;
		pose2d.x = currentPose.pose.position.x;
		pose2d.y = currentPose.pose.position.y;
		pose2d.theta = tf2::getYaw(currentPose.pose.orientation);

		if (!isCollisionFree(relative_yaw_, msgTwist.twist, pose2d))
		{
			stopRobot();
			RCLCPP_WARN(node_->get_logger(), "Collision Ahead - Exiting Spin");
			return Status::SUCCEEDED;
		}

		if (vel_pub_)
		{
			vel_pub_->publish(msgTwist);
		}
		else
		{
			vel_unstamped_pub_->publish(msgTwist.twist);
		}

		return Status::RUNNING;
	}

	bool SpinToPath::isCollisionFree(const double& RelativeYaw,
		const geometry_msgs::msg::Twist& CmdVel, geometry_msgs::msg::Pose2D& Pose2d)
	{
		// Simulate ahead by simulate_ahead_time_ in cycle_frequency_ increments
		int cycle_count = 0;
		double sim_position_change;
		const int max_cycle_count = static_cast<int>(cycle_frequency_ * simulate_ahead_time_);

		while (cycle_count < max_cycle_count)
		{
			sim_position_change = CmdVel.angular.z * (cycle_count / cycle_frequency_);
			Pose2d.theta += sim_position_change;
			cycle_count++;

			if (abs(RelativeYaw) - abs(sim_position_change) <= 0.)
			{
				break;
			}

			if (!collision_checker_->isCollisionFree(Pose2d))
			{
				return false;
			}
		}
		return true;
	}

} // namespace whi_nav2_bt_actions_server

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(whi_nav2_bt_actions_server::SpinToPath, whi_nav2_bt_actions_server::BaseAction)
