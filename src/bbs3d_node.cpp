// Copyright 2026 Iwana Robotics
#include <iostream>

#include "bbs3d_ros2/bbs3d_node.hpp"

namespace bbs3d_ros2
{

// Step 3 RED skeleton: prints one line to stdout so launch_testing has output
// to scan, but does NOT emit "[ROS2] 3D-BBS initialized" so the smoke test
// fails by timeout. GREEN replaces this with the upstream port.
Bbs3dNode::Bbs3dNode(const rclcpp::NodeOptions & options)
: Node("bbs3d_ros2_node", options)
{
  std::cout << "[ROS2] skeleton constructed (RED state)" << std::endl;
}

Bbs3dNode::~Bbs3dNode() = default;

}  // namespace bbs3d_ros2
