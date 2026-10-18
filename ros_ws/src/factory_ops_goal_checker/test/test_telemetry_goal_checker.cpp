#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "factory_ops_goal_checker/telemetry_goal_checker.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{

geometry_msgs::msg::Pose pose(double x, double yaw)
{
  geometry_msgs::msg::Pose value;
  value.position.x = x;
  value.orientation.z = std::sin(yaw / 2.0);
  value.orientation.w = std::cos(yaw / 2.0);
  return value;
}

class GoalCheckerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(GoalCheckerTest, MatchesSimpleGoalCheckerStatefulSemanticsAndReturnedTolerances)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("goal_checker.stateful", true),
    rclcpp::Parameter("goal_checker.xy_goal_tolerance", 0.06),
    rclcpp::Parameter("goal_checker.yaw_goal_tolerance", 0.15),
  });
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_checker_test", options);
  auto stock_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("stock_goal_checker_test", options);
  std::vector<diagnostic_msgs::msg::DiagnosticArray> traces;
  auto subscriber = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "controller_cycle_trace", 100,
    [&traces](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {traces.push_back(*message);});
  factory_ops_goal_checker::TelemetryGoalChecker checker;
  nav2_controller::SimpleGoalChecker stock;
  checker.initialize(node, "goal_checker", nullptr);
  stock.initialize(stock_node, "goal_checker", nullptr);
  geometry_msgs::msg::Twist stopped;
  const auto goal = pose(0.0, 0.0);
  for (const auto & query : {pose(0.061, 0.0), pose(0.059, -0.16), pose(0.061, -0.14)}) {
    EXPECT_EQ(stock.isGoalReached(query, goal, stopped), checker.isGoalReached(query, goal, stopped));
  }
  checker.reset();
  stock.reset();
  EXPECT_EQ(stock.isGoalReached(pose(0.061, 0.0), goal, stopped),
    checker.isGoalReached(pose(0.061, 0.0), goal, stopped));
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  ASSERT_TRUE(checker.getTolerances(pose_tolerance, velocity_tolerance));
  geometry_msgs::msg::Pose stock_pose_tolerance;
  geometry_msgs::msg::Twist stock_velocity_tolerance;
  ASSERT_TRUE(stock.getTolerances(stock_pose_tolerance, stock_velocity_tolerance));
  EXPECT_EQ(stock_pose_tolerance, pose_tolerance);
  EXPECT_EQ(stock_velocity_tolerance, velocity_tolerance);
  EXPECT_DOUBLE_EQ(0.06, pose_tolerance.position.x);
  EXPECT_DOUBLE_EQ(0.06, pose_tolerance.position.y);
  EXPECT_NEAR(0.15, 2.0 * std::asin(pose_tolerance.orientation.z), 1e-12);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  for (int i = 0; i < 20 && traces.size() < 7; ++i) {
    executor.spin_some();
  }
  ASSERT_EQ(7u, traces.size());
  std::uint64_t expected_sequence = 1;
  for (const auto & trace : traces) {
    ASSERT_EQ(1u, trace.status.size());
    const auto & status = trace.status.front();
    std::vector<std::string> keys;
    for (const auto & item : status.values) {keys.push_back(item.key);}
    EXPECT_NE(keys.end(), std::find(keys.begin(), keys.end(), "schema_version"));
    auto sequence = std::find_if(status.values.begin(), status.values.end(),
      [](const auto & item) {return item.key == "plugin_call_sequence";});
    ASSERT_NE(status.values.end(), sequence);
    EXPECT_EQ(expected_sequence++, std::stoull(sequence->value));
    if (status.message != "is_goal_reached") {
      EXPECT_EQ(keys.end(), std::find(keys.begin(), keys.end(), "query_x_m"));
      EXPECT_EQ(keys.end(), std::find(keys.begin(), keys.end(), "check_xy_before"));
    }
  }
}

}  // namespace
