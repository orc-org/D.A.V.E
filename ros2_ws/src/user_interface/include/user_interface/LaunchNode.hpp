#pragma once

#include "rclcpp/rclcpp.hpp"

/*
	LaunchNode
	------------------------
	A ROS2 Node that is responsible for passing infromation between our user interface and 
	the rest of our systen
	It is also responsible for launching and terminating all of our other nodes as the user requests it
*/
class LaunchNode : public rclcpp::Node
{
public: 

    LaunchNode();

private:


};