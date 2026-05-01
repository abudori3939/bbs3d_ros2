// Copyright 2026 Iwana Robotics
#pragma once

#include <rclcpp/rclcpp.hpp>

namespace bbs3d_ros2
{

class Bbs3dNode : public rclcpp::Node
{
public:
  explicit Bbs3dNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Bbs3dNode() override;
};

}  // namespace bbs3d_ros2
