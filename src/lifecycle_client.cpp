/*
MIT License

Copyright (c) 2020 Bytes Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/// For debug printing
const std::array<std::string, 17> state_map{
  "PRIMARY_STATE_UNKNOWN", "PRIMARY_STATE_UNCONFIGURED",
  "PRIMARY_STATE_INACTIVE", "PRIMARY_STATE_ACTIVE",
  "PRIMARY_STATE_FINALIZED", "TRANSITION_STATE_CONFIGURING",
  "", "", "", "", "",
  "TRANSITION_STATE_CONFIGURING", "TRANSITION_STATE_CLEANINGUP",
  "TRANSITION_STATE_SHUTTINGDOWN", "TRANSITION_STATE_ACTIVATING",
  "TRANSITION_STATE_DEACTIVATING", "TRANSITION_STATE_ERRORPROCESSING"};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("t260_lifecycle_client");

  std::string node_name = node->declare_parameter("server_node_name", "t260_node");
  bool autostart = node->declare_parameter("autostart", true);

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client =
    node->create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");

  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client =
    node->create_client<lifecycle_msgs::srv::GetState>(node_name + "/get_state");

  while (!get_state_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto get_state_request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto state = get_state_client->async_send_request(get_state_request);
  if (rclcpp::spin_until_future_complete(node, state) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"), "Current State: %s",
      state_map[state.get()->current_state.id].c_str());

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  if (autostart) {
    /// Transition to configured state
    auto change_state_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_state_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    auto response = change_state_client->async_send_request(change_state_request);
    if (rclcpp::spin_until_future_complete(node, response) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully transitioned to configured state!");

      /// Transitions to active state
      auto change_state_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
      change_state_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
      auto response = change_state_client->async_send_request(change_state_request);
      if (rclcpp::spin_until_future_complete(node, response) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully transitioned to active state!");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to activate T265 node");
      }

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to configure T265 node");
    }
  }

  rclcpp::shutdown();
  return 0;
}
