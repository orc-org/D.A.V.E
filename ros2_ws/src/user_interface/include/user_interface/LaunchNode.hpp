#pragma once

// #define RAYGUI_IMPLEMENTATION
// #include "raygui.h"
#include <iostream>
#include <raylib.h>

#include "rclcpp/rclcpp.hpp"
#include "ResourceManager.hpp"
#include "InstrumentPanel.hpp"
#include "UIConfig.hpp"

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

	/*
		DisplayGUI
		------------
		function that will handle the drawing and updating of our graphical user interface
		this will be run in it's own thread so as to not interfere with any ros2 timers
	*/
	void DisplayGUI();

private:


};