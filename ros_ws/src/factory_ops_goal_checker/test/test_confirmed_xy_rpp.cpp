#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "factory_ops_goal_checker/confirmed_xy_regulated_pure_pursuit_controller.hpp"
#include "factory_ops_goal_checker/telemetry_goal_checker.hpp"
#include "factory_ops_goal_checker/telemetry_regulated_pure_pursuit_controller.hpp"
#include "factory_ops_goal_checker/trace_contract.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace
{

class LegacyProbe : public factory_ops_goal_checker::TelemetryRegulatedPurePursuitController
{
public:
  auto state() const {return std::make_tuple(is_rotating_to_heading_, has_reached_xy_tolerance_);}
};

class ConfirmedProbe : public factory_ops_goal_checker::ConfirmedXyRegulatedPurePursuitController
{
public:
  explicit ConfirmedProbe(bool fail = false) : fail_(fail) {}
  auto state() const {return std::make_tuple(is_rotating_to_heading_, has_reached_xy_tolerance_);}

protected:
  void telemetryFaultPoint() override
  {
    if (fail_) {throw std::runtime_error("forced telemetry failure");}
  }

private:
  bool fail_;
};

class ConfirmedXyRppTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

nav_msgs::msg::Path straightPlan(double goal_x = 1.0, double goal_yaw = 0.0)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x : {0.0, goal_x / 2.0, goal_x}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    const double yaw = x == goal_x ? goal_yaw : 0.0;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::PoseStamped robotPose(double x, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

geometry_msgs::msg::Pose goalPose(double x = 1.0, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

TEST_F(ConfirmedXyRppTest, ReleasesOnlyUnconfirmedStaleLatchAndPreservesConfirmedRotation)
{
  rclcpp::NodeOptions costmap_options;
  costmap_options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("width", 10), rclcpp::Parameter("height", 10),
    rclcpp::Parameter("resolution", 0.1), rclcpp::Parameter("origin_x", -5.0),
    rclcpp::Parameter("origin_y", -5.0), rclcpp::Parameter("global_frame", "map"),
    rclcpp::Parameter("robot_base_frame", "base_link"), rclcpp::Parameter("robot_radius", 0.2),
  });
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_options);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, costmap->configure().id());
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({
    rclcpp::Parameter("Goal.stateful", true),
    rclcpp::Parameter("Goal.xy_goal_tolerance", 0.06),
    rclcpp::Parameter("Goal.yaw_goal_tolerance", 0.15),
    rclcpp::Parameter("Goal.require_stopped_velocity", true),
  });
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("confirmed_xy_rpp", node_options);
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 0.95;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));

  factory_ops_goal_checker::TelemetryGoalChecker goal_checker;
  goal_checker.initialize(node, "Goal", costmap);
  LegacyProbe legacy;
  ConfirmedProbe confirmed;
  legacy.configure(node, "Legacy", tf, costmap);
  confirmed.configure(node, "Confirmed", tf, costmap);
  legacy.activate();
  confirmed.activate();
  geometry_msgs::msg::Twist velocity;

  legacy.setPlan(straightPlan());
  confirmed.setPlan(straightPlan());
  const auto near_goal = robotPose(0.93);
  const auto legacy_latched = legacy.computeVelocityCommands(near_goal, velocity, &goal_checker);
  const auto confirmed_latched =
    confirmed.computeVelocityCommands(near_goal, velocity, &goal_checker);
  EXPECT_DOUBLE_EQ(0.0, legacy_latched.twist.linear.x);
  EXPECT_DOUBLE_EQ(0.0, confirmed_latched.twist.linear.x);
  EXPECT_TRUE(std::get<1>(legacy.state()));
  EXPECT_TRUE(std::get<1>(confirmed.state()));
  EXPECT_FALSE(goal_checker.xyConfirmed());

  transform.transform.translation.x = 0.93;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  const auto outside_goal = robotPose(0.93);
  const auto legacy_stopped = legacy.computeVelocityCommands(outside_goal, velocity, &goal_checker);
  const auto confirmed_resumed =
    confirmed.computeVelocityCommands(outside_goal, velocity, &goal_checker);
  EXPECT_DOUBLE_EQ(0.0, legacy_stopped.twist.linear.x);
  EXPECT_GT(confirmed_resumed.twist.linear.x, 0.0);
  EXPECT_TRUE(std::get<1>(legacy.state()));
  EXPECT_FALSE(std::get<1>(confirmed.state()));

  geometry_msgs::msg::Twist stopped;
  EXPECT_FALSE(goal_checker.isGoalReached(robotPose(0.95).pose, goalPose(1.0, 0.3), stopped));
  EXPECT_TRUE(goal_checker.xyConfirmed());
  transform.transform.translation.x = 0.95;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  confirmed.setPlan(straightPlan(1.0, 0.3));
  const auto rotating = confirmed.computeVelocityCommands(robotPose(0.95), velocity, &goal_checker);
  EXPECT_DOUBLE_EQ(0.0, rotating.twist.linear.x);
  EXPECT_NE(0.0, rotating.twist.angular.z);
  EXPECT_TRUE(std::get<1>(confirmed.state()));
  EXPECT_TRUE(goal_checker.xyConfirmed());
  transform.transform.translation.x = 0.93;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  const auto rotating_outside_xy =
    confirmed.computeVelocityCommands(robotPose(0.93), velocity, &goal_checker);
  EXPECT_DOUBLE_EQ(0.0, rotating_outside_xy.twist.linear.x);
  EXPECT_NE(0.0, rotating_outside_xy.twist.angular.z);
  EXPECT_TRUE(std::get<1>(confirmed.state()));

  goal_checker.reset();
  confirmed.setPlan(straightPlan());
  transform.transform.translation.x = 0.95;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.05784344698;
  moving.angular.z = -0.38109283967;
  EXPECT_FALSE(goal_checker.isGoalReached(robotPose(0.95).pose, goalPose(), moving));
  ASSERT_TRUE(goal_checker.terminalStopRequested());
  const auto terminal_brake =
    confirmed.computeVelocityCommands(robotPose(0.95), moving, &goal_checker);
  EXPECT_DOUBLE_EQ(0.0, terminal_brake.twist.linear.x);
  EXPECT_DOUBLE_EQ(0.0, terminal_brake.twist.linear.y);
  EXPECT_DOUBLE_EQ(0.0, terminal_brake.twist.angular.z);

  EXPECT_TRUE(goal_checker.isGoalReached(robotPose(0.93).pose, goalPose(), velocity));
  EXPECT_TRUE(goal_checker.xyConfirmed());
  EXPECT_FALSE(goal_checker.isGoalReached(robotPose(0.84).pose, goalPose(), velocity));
  EXPECT_FALSE(goal_checker.terminalStopRequested());
  EXPECT_FALSE(goal_checker.xyConfirmed());
  transform.transform.translation.x = 0.84;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  const auto drift_reapproach =
    confirmed.computeVelocityCommands(robotPose(0.84), velocity, &goal_checker);
  EXPECT_GT(drift_reapproach.twist.linear.x, 0.0);
  EXPECT_FALSE(std::get<1>(confirmed.state()));

  transform.transform.translation.x = 0.95;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  EXPECT_FALSE(goal_checker.isGoalReached(robotPose(0.95).pose, goalPose(), moving));
  const auto terminal_brake_again =
    confirmed.computeVelocityCommands(robotPose(0.95), moving, &goal_checker);
  EXPECT_DOUBLE_EQ(0.0, terminal_brake_again.twist.linear.x);
  EXPECT_DOUBLE_EQ(0.0, terminal_brake_again.twist.linear.y);
  EXPECT_DOUBLE_EQ(0.0, terminal_brake_again.twist.angular.z);
  EXPECT_TRUE(goal_checker.isGoalReached(robotPose(0.95).pose, goalPose(), velocity));
  EXPECT_FALSE(goal_checker.terminalStopRequested());

  confirmed.setPlan(straightPlan());
  goal_checker.reset();
  EXPECT_FALSE(goal_checker.xyConfirmed());
  transform.transform.translation.x = 0.93;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  const auto new_goal_resumed =
    confirmed.computeVelocityCommands(outside_goal, velocity, &goal_checker);
  EXPECT_GT(new_goal_resumed.twist.linear.x, 0.0);
  EXPECT_FALSE(std::get<1>(confirmed.state()));

  confirmed.deactivate();
  legacy.deactivate();
}

TEST_F(ConfirmedXyRppTest, RejectsWrongGoalCheckerAndTelemetryFailureDoesNotChangeCommand)
{
  rclcpp::NodeOptions costmap_options;
  costmap_options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("width", 10), rclcpp::Parameter("height", 10),
    rclcpp::Parameter("resolution", 0.1), rclcpp::Parameter("origin_x", -5.0),
    rclcpp::Parameter("origin_y", -5.0), rclcpp::Parameter("global_frame", "map"),
    rclcpp::Parameter("robot_base_frame", "base_link"), rclcpp::Parameter("robot_radius", 0.2),
  });
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_options);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, costmap->configure().id());
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("confirmed_xy_failures");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));

  factory_ops_goal_checker::TelemetryGoalChecker goal_checker;
  goal_checker.initialize(node, "Goal", costmap);
  ConfirmedProbe normal;
  ConfirmedProbe failing(true);
  normal.configure(node, "Normal", tf, costmap);
  failing.configure(node, "Failing", tf, costmap);
  normal.activate();
  failing.activate();
  std::vector<diagnostic_msgs::msg::DiagnosticArray> traces;
  auto subscriber = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "controller_cycle_trace", factory_ops_goal_checker::traceQoS(),
    [&traces](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
      traces.push_back(*message);
    });
  normal.setPlan(straightPlan());
  failing.setPlan(straightPlan());
  geometry_msgs::msg::Twist velocity;
  const auto pose = robotPose(0.0);
  EXPECT_EQ(
    normal.computeVelocityCommands(pose, velocity, &goal_checker),
    failing.computeVelocityCommands(pose, velocity, &goal_checker));

  nav2_controller::SimpleGoalChecker wrong_goal_checker;
  wrong_goal_checker.initialize(node, "WrongGoal", costmap);
  normal.setPlan(straightPlan());
  EXPECT_THROW(
    static_cast<void>(normal.computeVelocityCommands(pose, velocity, &wrong_goal_checker)),
    std::invalid_argument);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  for (int i = 0; i < 100 && traces.size() < 4; ++i) {
    executor.spin_some(std::chrono::milliseconds(20));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool found_rejection = false;
  for (const auto & trace : traces) {
    for (const auto & status : trace.status) {
      std::string outcome;
      std::string error_message;
      for (const auto & item : status.values) {
        if (item.key == "outcome") {outcome = item.value;}
        if (item.key == "error_message") {error_message = item.value;}
      }
      if (status.name == "Normal" && outcome == "threw" &&
        error_message.find("requires TelemetryGoalChecker") != std::string::npos)
      {
        found_rejection = true;
      }
    }
  }
  EXPECT_TRUE(found_rejection);

  failing.deactivate();
  normal.deactivate();
}

}  // namespace
