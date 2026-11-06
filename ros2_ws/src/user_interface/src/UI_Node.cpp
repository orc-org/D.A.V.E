#include "user_interface/UI_Node.hpp"
#include <iostream>

// #define RAYGUI_IMPLEMENTATION
// #include "raygui.h"



UI_Node::UI_Node() : rclcpp::Node("UI_Node")
{
    std::cout << "Great Success" << std::endl;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UI_Node>());
  rclcpp::shutdown();
  return 0;
}