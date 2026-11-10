/*
 * Copyright (c) 2026 Airvolute s.r.o.
 * All rights reserved.
 *
 * This file is part of the DroneCore Platform.
 *
 * Licensed under the Airvolute DroneCore Platform License (ADPL), Version 1.0.
 * You may use, modify, and build this file only in accordance with the ADPL.
 *
 * Permitted use is limited to:
 *   - Airvolute Hardware
 *   - the DroneCore Platform
 *   - Derived Systems built on, integrated with, or dependent on the DroneCore Platform
 *
 * Modified versions may be used and deployed only within Airvolute Hardware,
 * the DroneCore Platform, or Derived Systems. Distribution of modified versions
 * is permitted only as part of a complete system or product and not as a
 * standalone software offering, unless explicitly authorized in writing by
 * Airvolute s.r.o.
 *
 * The Software and all related intellectual property rights are owned by
 * Airvolute s.r.o. This license grants usage rights only and does not transfer
 * ownership.
 *
 * The Software is provided "as is" and "as available", without warranties of
 * any kind, to the maximum extent permitted by applicable law.
 *
 * Contact: info@airvolute.com
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "joint_trajectory_controller/interpolation_methods.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{
using Point = trajectory_msgs::msg::JointTrajectoryPoint;
using Message = trajectory_msgs::msg::JointTrajectory;
using Trajectory = joint_trajectory_controller::Trajectory;

/** @brief Use a nonzero ROS epoch without initializing ROS or touching the network. */
rclcpp::Time Stamp(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>((100.0 + seconds) * 1e9), RCL_ROS_TIME);
}

/** @brief Analytic four-phase constant-jerk move, held at rest once complete. */
Point Exact(double t)
{
  double p = 0.0, v = 0.0, a = 0.0;
  for (const double jerk : {1.0, -1.0, -1.0, 1.0}) {
    const double dt = std::clamp(t, 0.0, 0.2);
    p += v * dt + a * dt * dt / 2.0 + jerk * dt * dt * dt / 6.0;
    v += a * dt + jerk * dt * dt / 2.0;
    a += jerk * dt;
    t -= dt;
  }
  Point point;
  point.positions = {p};
  point.velocities = {v};
  point.accelerations = {a};
  return point;
}

/** @brief Export the jerk boundaries of the remaining move at its planner timestamp. */
std::shared_ptr<Message> Preview(double start)
{
  auto message = std::make_shared<Message>();
  message->header.stamp = Stamp(start);
  message->joint_names = {"joint"};
  std::vector<double> times{start};
  for (const double boundary : {0.2, 0.4, 0.6, 0.8}) {
    if (boundary > start) {
      times.push_back(boundary);
    }
  }
  for (const double time : times) {
    auto point = Exact(time);
    point.time_from_start = rclcpp::Duration::from_seconds(time - start);
    message->points.push_back(point);
  }
  return message;
}

/** @brief Sample the installed controller interpolator against an independent analytic answer. */
void Check(Trajectory & trajectory, double time)
{
  Point actual;
  joint_trajectory_controller::TrajectoryPointConstIter begin, end;
  ASSERT_TRUE(trajectory.sample(
    Stamp(time), joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION, actual,
    begin, end));
  const auto expected = Exact(time);
  ASSERT_EQ(actual.positions.size(), 1u);
  EXPECT_NEAR(actual.positions[0], expected.positions[0], 1e-8);
  EXPECT_NEAR(actual.velocities[0], expected.velocities[0], 1e-7);
  EXPECT_NEAR(actual.accelerations[0], expected.accelerations[0], 1e-6);
  EXPECT_LE(std::abs(actual.accelerations[0]), 0.2 + 1e-6);
}

TEST(IpInterpolation, PhaseBoundariesPreserveDynamicsAt125Hz)
{
  Trajectory trajectory;
  trajectory.update(Preview(0.0));
  trajectory.set_point_before_trajectory_msg(Stamp(0.0), Exact(0.0));
  for (int i = 0; i < 150; ++i) {
    Check(trajectory, i * 0.008);
  }
}

TEST(IpInterpolation, DelayedReplacementsKeepPlannerClock)
{
  Trajectory trajectory;
  trajectory.update(Preview(0.0));
  trajectory.set_point_before_trajectory_msg(Stamp(0.0), Exact(0.0));
  std::array<double, 4> planner_times{0.05, 0.18, 0.31, 0.50};
  std::array<double, 4> delays{0.006, 0.080, 0.025, 0.090};
  size_t next = 0;
  for (int i = 0; i < 150; ++i) {
    const double now = i * 0.008;
    if (next < planner_times.size() && now >= planner_times[next] + delays[next]) {
      trajectory.update(Preview(planner_times[next]));
      trajectory.set_point_before_trajectory_msg(Stamp(now), Exact(now - 0.008));
      ++next;
    }
    Check(trajectory, now);
  }
  EXPECT_EQ(next, planner_times.size());
}

TEST(IpInterpolation, MissingUpdatesEndAtZeroVelocityAndAcceleration)
{
  Trajectory trajectory;
  trajectory.update(Preview(0.0));
  trajectory.set_point_before_trajectory_msg(Stamp(0.0), Exact(0.0));
  for (const double time : {0.0, 0.4, 0.8, 1.0, 2.0}) {
    Check(trajectory, time);
  }
}
}  // namespace
