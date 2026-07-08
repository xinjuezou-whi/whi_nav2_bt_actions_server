/******************************************************************
bt action plugin of locomotion offset

Features:
- adjust locomotion offset
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-07-07: Initial version
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/action/locomotion_offset.hpp"
#include "whi_nav2_bt_actions_server/base_action.hpp"

#include <memory>
#include <string>

namespace whi_nav2_bt_actions_server
{
	using LocomotionOffsetAction = whi_interfaces::action::LocomotionOffset;

	class LocomotionOffset : public BaseActionT<LocomotionOffsetAction>
	{
	public:
		using LocomotionOffsetGoal = LocomotionOffsetAction::Goal;
		using LocomotionOffsetResult = LocomotionOffsetAction::Result;

		LocomotionOffset();
		~LocomotionOffset();

	public:
		Status onRun(const std::shared_ptr<const LocomotionOffsetGoal> Command) override;
		void onConfigure() override;
		Status onCycleUpdate() override;

	protected:
		void rotateToApproach(const geometry_msgs::msg::PoseStamped& CurrentPose);
		void moveToMidTarget(const geometry_msgs::msg::PoseStamped& CurrentPose);
		void rotateToMidTarget(const geometry_msgs::msg::PoseStamped& CurrentPose);
		void moveToTarget(const geometry_msgs::msg::PoseStamped& CurrentPose);
		void rotateToFinal(const geometry_msgs::msg::PoseStamped& CurrentPose);
		bool isCollisionFree(const geometry_msgs::msg::Twist& CmdVel,
			const geometry_msgs::msg::PoseStamped& CurrentPose);

		LocomotionOffsetAction::Feedback::SharedPtr feedback_{ nullptr };

		double min_rotational_vel_{ 0.1 };
		double max_rotational_vel_{ 1.0 };
		double min_linear_vel_{ 0.1 };
		double max_linear_vel_{ 1.0 };
		double simulate_ahead_time_{ 2.0 };
		enum State { IDLE = 0, ROTATE_TO_APPROACH, MOVE_TO_MID_TARGET, ROTATE_TO_MID_TARGET, MOVE_TO_TARGET, FINAL_ROTATE, FINISHED, CANCELED };
		State state_{ State::IDLE };
		geometry_msgs::msg::PoseStamped target_pose_;
		std::shared_ptr<geometry_msgs::msg::PoseStamped> mid_target_pose_{ nullptr };
		bool move_backward_{ false };
		double approach_yaw_{ 0.0 };
		double position_tolerance_{ 0.005 };
		double yaw_tolerance_{ 2.0 * M_PI / 180.0 };
		double default_timeout_{ 50.0 }; // seconds
		double timeout_{ 50.0 }; // seconds
		rclcpp::Time start_time_;

		Twist cmd_twist_;
	};

} // namespace whi_nav2_bt_actions_server
