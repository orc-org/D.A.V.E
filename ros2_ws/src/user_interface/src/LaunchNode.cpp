#include "user_interface/LaunchNode.hpp"
#include <iostream>

// #define RAYGUI_IMPLEMENTATION
// #include "raygui.h"


LaunchNode::LaunchNode() : rclcpp::Node("LaunchNode")
{
	std::cout << "Great Success" << std::endl;
}
