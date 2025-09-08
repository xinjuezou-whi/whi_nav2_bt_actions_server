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
		SpinToPath();
		~SpinToPath();

		Status onRun(const std::shared_ptr<const SpinToPathAction::Goal> Command) override;
		void onConfigure() override;
		Status onCycleUpdate() override;

	protected:
		bool isCollisionFree(
			const double &distance,
			geometry_msgs::msg::Twist *cmd_vel,
			geometry_msgs::msg::Pose2D &pose2d);

		SpinToPathAction::Feedback::SharedPtr feedback_;

		double min_rotational_vel_;
		double max_rotational_vel_;
		double rotational_acc_lim_;
		double cmd_yaw_;
		double prev_yaw_;
		double relative_yaw_;
		double simulate_ahead_time_;
	};

} // namespace whi_nav2_bt_actions_server
