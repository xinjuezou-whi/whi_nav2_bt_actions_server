/******************************************************************
bt action plugin of spin to path

Features:
- spin to align with the planned path
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
#include "../base_action.hpp"

#include <memory>
#include <string>

namespace whi_nav2_bt_actions_server
{
	using SpinToPathAction = whi_interfaces::action::SpinToPath;

	class SpinToPath : public BaseActionT<SpinToPathAction>
	{
	public:
		using SpinToPathGoal = SpinToPathAction::Goal;
		using SpinToPathResult = SpinToPathAction::Result;

		SpinToPath();
		~SpinToPath();

	public:
		ResultStatus onRun(const std::shared_ptr<const SpinToPathGoal> Command) override;
		void onConfigure() override;
		ResultStatus onCycleUpdate() override;
		CostmapInfoType getResourceInfo() override { return CostmapInfoType::LOCAL; };

	protected:
		bool isCollisionFree(const double& RelativeYaw, const geometry_msgs::msg::Twist& CmdVel,
			geometry_msgs::msg::Pose2D& Pose2d);

		SpinToPathAction::Feedback::SharedPtr feedback_{ nullptr };

		double min_rotational_vel_{ 0.1 };
		double max_rotational_vel_{ 1.0 };
		double rotational_acc_lim_{ 1.57 };
		double cmd_yaw_{ 0.0 };
		double prev_yaw_{ 0.0 };
		double relative_yaw_{ 0.0 };
		double simulate_ahead_time_{ 2.0 };
	};

} // namespace whi_nav2_bt_actions_server
