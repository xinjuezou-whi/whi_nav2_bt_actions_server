/******************************************************************
nav2 bt actions server
it is a interface of nav2 actions

Features:
- plugin spin to path
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-09-08: Initial version
2025-xx-xx: xxx
******************************************************************/
#include "whi_nav2_bt_actions_server/bt_actions_server.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	/// node version and copyright announcement
	std::cout << "\nWHI nav2 bt actions server VERSION 00.00.1" << std::endl;
	std::cout << "Copyright © 2025-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	auto node = std::make_shared<whi_nav2_bt_actions_server::BtActionsServer>();
	rclcpp::spin(node->get_node_base_interface());
	rclcpp::shutdown();

	return 0;
}
