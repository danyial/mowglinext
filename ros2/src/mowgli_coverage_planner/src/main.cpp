// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mowgli_coverage_planner/coverage_planner_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_coverage_planner::CoveragePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
