#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <typeinfo>
#include <vector>

#include "factory_ops_goal_checker/telemetry_regulated_pure_pursuit_controller.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace
{
class StockProbe : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  auto state() const {return std::make_tuple(is_rotating_to_heading_, has_reached_xy_tolerance_, goal_dist_tol_);}
};

class TraceProbe : public factory_ops_goal_checker::TelemetryRegulatedPurePursuitController
{
public:
  explicit TraceProbe(bool fail = false) : fail_(fail) {}
  auto state() const {return std::make_tuple(is_rotating_to_heading_, has_reached_xy_tolerance_, goal_dist_tol_);}
protected:
  void telemetryFaultPoint() override {if (fail_) {throw std::runtime_error("forced telemetry failure");}}
private:
  bool fail_;
};

class RppDifferentialTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

nav_msgs::msg::Path straightPlan()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x : {0.0, 0.5, 1.0}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

TEST_F(RppDifferentialTest, ConfiguredWrapperMatchesStockAndPreservesOriginalFailures)
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
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("rpp_differential");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  StockProbe stock;
  TraceProbe trace(true);
  nav2_controller::SimpleGoalChecker goal_checker;
  goal_checker.initialize(node, "Goal", costmap);
  stock.configure(node, "Stock", tf, costmap);
  trace.configure(node, "Trace", tf, costmap);
  stock.activate(); trace.activate();
  const auto plan = straightPlan();
  stock.setPlan(plan); trace.setPlan(plan);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map"; pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist velocity;
  const auto expected = stock.computeVelocityCommands(pose, velocity, &goal_checker);
  const auto actual = trace.computeVelocityCommands(pose, velocity, &goal_checker);
  EXPECT_EQ(expected, actual);
  EXPECT_EQ(stock.state(), trace.state());

  nav_msgs::msg::Path invalid; invalid.header.frame_id = "map";
  stock.setPlan(invalid); trace.setPlan(invalid);
  std::string stock_type, stock_message, trace_type, trace_message;
  try {static_cast<void>(stock.computeVelocityCommands(pose, velocity, &goal_checker));}
  catch (const std::exception & error) {stock_type = typeid(error).name(); stock_message = error.what();}
  try {static_cast<void>(trace.computeVelocityCommands(pose, velocity, &goal_checker));}
  catch (const std::exception & error) {trace_type = typeid(error).name(); trace_message = error.what();}
  EXPECT_FALSE(stock_type.empty());
  EXPECT_EQ(stock_type, trace_type);
  EXPECT_EQ(stock_message, trace_message);
  EXPECT_EQ(stock.state(), trace.state());
  trace.deactivate(); stock.deactivate();
}
}  // namespace
