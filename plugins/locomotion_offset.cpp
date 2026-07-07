/******************************************************************
bt action plugin of locomotion offset

Features:
- adjust locomotion offset
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_nav2_bt_actions_server/plugins/locomotion_offset.hpp"

#include <nav2_util/node_utils.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <tf2/utils.hpp>
#include <angles/angles.h>
#include <geometry_msgs/msg/point.hpp>

namespace whi_nav2_bt_actions_server
{
	LocomotionOffset::LocomotionOffset()
		: BaseActionT<LocomotionOffsetAction>()
		, feedback_(std::make_shared<LocomotionOffsetAction::Feedback>())
	{
		/// node version and copyright announcement
		std::cout << "\nWHI bt action locomotion offset VERSION 00.01.1" << std::endl;
		std::cout << "Copyright © 2025-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
	}

	LocomotionOffset::~LocomotionOffset() {}

	void LocomotionOffset::onConfigure()
	{
		auto node = node_.lock();
		if (!node)
		{
			throw std::runtime_error{"Failed to lock node"};
		}

		nav2_util::declare_parameter_if_not_declared(node, action_name_ + ".min_rotational_vel", rclcpp::ParameterValue(0.1));
		node->get_parameter(action_name_ + ".min_rotational_vel", min_rotational_vel_);

		nav2_util::declare_parameter_if_not_declared(node, action_name_ + ".max_rotational_vel", rclcpp::ParameterValue(1.0));
		node->get_parameter(action_name_ + ".max_rotational_vel", max_rotational_vel_);

		nav2_util::declare_parameter_if_not_declared(node, action_name_ + ".min_linear_vel", rclcpp::ParameterValue(0.1));
		node->get_parameter(action_name_ + ".min_linear_vel", min_linear_vel_);

		nav2_util::declare_parameter_if_not_declared(node, action_name_ + ".max_linear_vel", rclcpp::ParameterValue(1.0));
		node->get_parameter(action_name_ + ".max_linear_vel", max_linear_vel_);

		nav2_util::declare_parameter_if_not_declared(node, action_name_ + ".position_tolerance", rclcpp::ParameterValue(0.005));
		node->get_parameter(action_name_ + ".position_tolerance", position_tolerance_);

		nav2_util::declare_parameter_if_not_declared(node, action_name_ + ".yaw_tolerance", rclcpp::ParameterValue(2.0));
		double yawTolerance;
		node->get_parameter(action_name_ + ".yaw_tolerance", yawTolerance);
		yaw_tolerance_ = angles::from_degrees(yawTolerance);

		nav2_util::declare_parameter_if_not_declared(node, action_name_ + ".check_collision", rclcpp::ParameterValue(true));
		node->get_parameter(action_name_ + ".check_collision", check_collision_);
	}

	bool transformTo(const geometry_msgs::msg::PoseStamped& SrcPose,
		geometry_msgs::msg::PoseStamped& DstPose,
		const std::string& SrcFrame, const std::string& DstFrame,
		tf2_ros::Buffer& TfBuffer, const double TransformTimeout)
	{
		static rclcpp::Logger logger = rclcpp::get_logger("transformTo");
		
		geometry_msgs::msg::PoseStamped src = SrcPose;
		src.header.frame_id = SrcFrame;
		src.header.stamp = rclcpp::Time();

		try
		{
			DstPose = TfBuffer.transform(src, DstFrame, tf2::durationFromSec(TransformTimeout));

			return true;
		}
		catch (tf2::LookupException& ex)
		{
			RCLCPP_ERROR(logger, "No Transform available Error looking up robot pose: %s\n",
				ex.what());
		}
		catch (tf2::ConnectivityException& ex)
		{
			RCLCPP_ERROR(logger, "Connectivity Error looking up robot pose: %s\n", ex.what());
		}
		catch (tf2::ExtrapolationException& ex)
		{
			RCLCPP_ERROR(logger, "Extrapolation Error looking up robot pose: %s\n", ex.what());
		}
		catch (tf2::TimeoutException& ex)
		{
			RCLCPP_ERROR(logger, "Transform timeout with tolerance: %.4f", TransformTimeout);
		}
		catch (tf2::TransformException& ex)
		{
			RCLCPP_ERROR(logger, "Failed to transform from %s to %s",
				SrcFrame.c_str(), DstFrame.c_str());
		}

		return false;
	}

	Status LocomotionOffset::onRun(const std::shared_ptr<const LocomotionOffsetGoal> Command)
	{
		geometry_msgs::msg::PoseStamped currentPose;
		if (!nav2_util::getCurrentPose(currentPose, *tf_, global_frame_, robot_base_frame_,
			transform_tolerance_))
		{
			std::string errorMsg("Current robot pose is not available.");
			RCLCPP_ERROR(logger_, errorMsg.c_str());

			return Status::FAILED;
		}

		target_pose_ = currentPose;

		const double xOffset = Command->offset.pose.position.x;
		const double yOffset = Command->offset.pose.position.y;
		const double yawOffset = tf2::getYaw(Command->offset.pose.orientation);
		const double currentYaw = tf2::getYaw(currentPose.pose.orientation);

		// compute target position in map frame
		target_pose_.pose.position.x += cos(currentYaw) * xOffset - sin(currentYaw) * yOffset;
		target_pose_.pose.position.y += sin(currentYaw) * xOffset + cos(currentYaw) * yOffset;

		tf2::Quaternion q;
		q.setRPY(0, 0, currentYaw + yawOffset);
		target_pose_.pose.orientation = tf2::toMsg(q);

		// initial value
		cmd_twist_.twist = geometry_msgs::msg::Twist();
		approach_yaw_ = 0.0;

		// check the direction to the target position
		double direction = atan2(yOffset, xOffset);
		double directionError = angles::shortest_angular_distance(0.0, direction);
		// Decide forward/backward
		move_backward_ = fabs(directionError) > M_PI_2;
		approach_yaw_ = angles::normalize_angle(currentYaw + direction);
		if (move_backward_)
		{
			approach_yaw_ = angles::normalize_angle(approach_yaw_ + M_PI);
		}
		// Always rotate first
		state_ = State::ROTATE_TO_APPROACH;

		RCLCPP_INFO(logger_, "LocomotionOffset started");

		return Status::SUCCEEDED;
	}

	Status LocomotionOffset::onCycleUpdate()
	{
		geometry_msgs::msg::PoseStamped currentPose;
		if (!nav2_util::getCurrentPose(currentPose, *tf_, global_frame_, robot_base_frame_,
			transform_tolerance_))
		{
			std::string errorMsg("Current robot pose is not available.");
			RCLCPP_ERROR(logger_, errorMsg.c_str());
			return Status::FAILED;
		}

		// reset cmd
		cmd_twist_.twist = geometry_msgs::msg::Twist();

		switch (state_)
		{
		case State::ROTATE_TO_APPROACH:
			rotateToApproach(currentPose);
			break;
		case State::MOVE_TO_TARGET:
			moveToTarget(currentPose);
			break;
		case State::FINAL_ROTATE:
			rotateToFinal(currentPose);
			break;
		case State::FINISHED:
			cmd_twist_.twist = geometry_msgs::msg::Twist();
			break;
		default:
			// undefined state
			return Status::FAILED;
			break;
		}

		cmd_twist_.header.stamp = clock_->now();
		if (vel_pub_)
		{
			vel_pub_->publish(cmd_twist_);
		}
		else
		{
			vel_unstamped_pub_->publish(cmd_twist_.twist);
		}

		if (state_ == State::FINISHED)
		{
			return Status::SUCCEEDED;
		}
		else
		{
			return Status::RUNNING;
		}
	}

	void LocomotionOffset::rotateToApproach(const geometry_msgs::msg::PoseStamped& CurrentPose)
	{
		const double currentYaw = tf2::getYaw(CurrentPose.pose.orientation);

		double error = angles::shortest_angular_distance(currentYaw, approach_yaw_);
		if (fabs(error) < yaw_tolerance_)
		{
			state_ = State::MOVE_TO_TARGET;
			return;
		}

		double magnitude = std::clamp(fabs(error), min_rotational_vel_, max_rotational_vel_);
		cmd_twist_.twist.angular.z = std::copysign(magnitude, error);
	}

	void LocomotionOffset::moveToTarget(const geometry_msgs::msg::PoseStamped& CurrentPose)
	{
		double dx = target_pose_.pose.position.x - CurrentPose.pose.position.x;
		double dy = target_pose_.pose.position.y - CurrentPose.pose.position.y;
		double distance = std::hypot(dx, dy);
		if (distance < position_tolerance_)
		{
			state_ = State::FINAL_ROTATE;
			return;
		}

		// Direction from robot to target in global frame
		double targetDirection = atan2(dy, dx);
		// Decide robot heading while moving
		double desiredHeading = targetDirection;
		if (move_backward_)
		{
			desiredHeading = angles::normalize_angle(targetDirection + M_PI);
		}
		double currentYaw = tf2::getYaw(CurrentPose.pose.orientation);
		double headingError = angles::shortest_angular_distance(currentYaw, desiredHeading);

		double linear = std::clamp(distance, min_linear_vel_, max_linear_vel_);
		double angular = std::clamp(fabs(headingError), min_rotational_vel_, max_rotational_vel_);
		cmd_twist_.twist.linear.x = move_backward_ ? -linear : linear;
		cmd_twist_.twist.angular.z = std::copysign(angular, headingError);
	}

	void LocomotionOffset::rotateToFinal(const geometry_msgs::msg::PoseStamped& CurrentPose)
	{
		const double targetYaw = tf2::getYaw(target_pose_.pose.orientation);
		const double currentYaw = tf2::getYaw(CurrentPose.pose.orientation);

		double error = angles::shortest_angular_distance(currentYaw, targetYaw);
		if (fabs(error) < yaw_tolerance_)
		{
			state_ = State::FINISHED;
			return;
		}

		double magnitude = std::clamp(fabs(error), min_rotational_vel_, max_rotational_vel_);
		cmd_twist_.twist.angular.z = std::copysign(magnitude, error);
	}

	bool LocomotionOffset::isCollisionFree(const double& RelativeYaw, const geometry_msgs::msg::Twist& CmdVel,
		geometry_msgs::msg::Pose2D& Pose2d)
	{
		if (!check_collision_)
		{
			return true;
		}

		// Simulate ahead by simulate_ahead_time_ in cycle_frequency_ increments
		int cycleCount = 0;
		double simPositionChange;
		const int maxCycleCount = static_cast<int>(cycle_frequency_ * simulate_ahead_time_);
		geometry_msgs::msg::Pose2D initPose = Pose2d;
		bool fetchData = true;

		while (cycleCount < maxCycleCount)
		{
			simPositionChange = CmdVel.angular.z * (cycleCount / cycle_frequency_);
			Pose2d.theta = initPose.theta + simPositionChange;
			cycleCount++;

			if (abs(RelativeYaw) - abs(simPositionChange) <= 0.)
			{
				break;
			}

			if (!collision_checker_->isCollisionFree(Pose2d, fetchData))
			{
				return false;
			}
			fetchData = false;
		}
		return true;
	}

} // namespace whi_nav2_bt_actions_server

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(whi_nav2_bt_actions_server::LocomotionOffset, whi_nav2_bt_actions_server::BaseAction)
