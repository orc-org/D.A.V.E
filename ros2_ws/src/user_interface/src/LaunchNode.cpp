#include "user_interface/LaunchNode.hpp"
#include <iostream>

// #define RAYGUI_IMPLEMENTATION
// #include "raygui.h"


LaunchNode::LaunchNode() : rclcpp::Node("LaunchNode")
{
	std::cout << "Great Success" << std::endl;
}

// DELETE main method, ros node should ot be an excecutable

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LaunchNode>());
	rclcpp::shutdown();
	return 0;
}