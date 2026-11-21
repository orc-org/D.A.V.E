#include <cstdio>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "serial_interfaces/srv/converse.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using Converse = serial_interfaces::srv::Converse;

class XLR_UI : public rclcpp::Node {

    std::string message;

public: 

    XLR_UI() : Node("XLR_UI") {
        ConverseClient = this->create_client<Converse> ("Converse");
    }

    void readInputLoop() {
        while (rclcpp::ok() && (std::cin >> message)) {
            converse(message);
        }
    }

private:

    void converse_response(const rclcpp::Client<Converse>::SharedFuture future){
        auto response = future.get();

        std::cout << response->incoming << '\n';
    }

    void converse(const std::string& msg){
        auto request = std::make_shared<Converse::Request>();
        request->outgoing = msg + '\n';

        ConverseClient->async_send_request(request, std::bind(&XLR_UI::converse_response, this, _1));
    }

    rclcpp::Client<Converse>::SharedPtr ConverseClient;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<XLR_UI>();
    std::thread input_thread(&XLR_UI::readInputLoop, node.get());
    input_thread.detach();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}