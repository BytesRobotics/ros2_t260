//
// Created by michael on 7/16/20.
//

#include "t260.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto t260_node = std::make_shared<T260>();
  rclcpp::spin(t260_node->get_node_base_interface());
  rclcpp::shutdown();
}
