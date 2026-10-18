#include "factory_ops_goal_checker/telemetry_goal_checker.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "factory_ops_goal_checker/trace_contract.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace factory_ops_goal_checker
{
namespace
{

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string number(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return stream.str();
}

std::string boolean(bool value)
{
  return value ? "true" : "false";
}

double yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double yawError(double query_yaw, double goal_yaw)
{
  const double delta = goal_yaw - query_yaw;
  return std::atan2(std::sin(delta), std::cos(delta));
}

}  // namespace

void TelemetryGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nav2_controller::SimpleGoalChecker::initialize(parent, plugin_name, costmap_ros);
  try {
    node_ = parent.lock();
    if (!node_) {return;}
    trace_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(kTraceTopic, traceQoS());
    trace_pub_->on_activate();
  } catch (...) {trace_pub_.reset();}
  publishSimpleTrace("initialize");
}

void TelemetryGoalChecker::reset()
{
  nav2_controller::SimpleGoalChecker::reset();
  {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    ++reset_epoch_;
  }
  publishSimpleTrace("reset");
}

bool TelemetryGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  const bool check_xy_before = check_xy_;
  const bool result = nav2_controller::SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  const bool check_xy_after = check_xy_;
  publishGoalTrace(query_pose, goal_pose, velocity, check_xy_before, check_xy_after, result);
  return result;
}

bool TelemetryGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  const bool result = nav2_controller::SimpleGoalChecker::getTolerances(pose_tolerance, vel_tolerance);
  publishToleranceTrace(pose_tolerance, result);
  return result;
}

void TelemetryGoalChecker::publishSimpleTrace(const char * event) noexcept
{
  publishTrace(event, {});
}

void TelemetryGoalChecker::publishGoalTrace(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity, bool check_xy_before,
  bool check_xy_after, bool result) noexcept
{
  try {
    const double query_yaw = yaw(query_pose.orientation);
    const double goal_yaw = yaw(goal_pose.orientation);
    publishTrace("is_goal_reached", {
      kv("query_x_m", number(query_pose.position.x)), kv("query_y_m", number(query_pose.position.y)),
      kv("query_yaw_rad", number(query_yaw)), kv("goal_x_m", number(goal_pose.position.x)),
      kv("goal_y_m", number(goal_pose.position.y)), kv("goal_yaw_rad", number(goal_yaw)),
      kv("xy_error_m", number(std::hypot(goal_pose.position.x - query_pose.position.x,
        goal_pose.position.y - query_pose.position.y))),
      kv("yaw_error_rad", number(yawError(query_yaw, goal_yaw))),
      kv("linear_x_m_s", number(velocity.linear.x)), kv("linear_y_m_s", number(velocity.linear.y)),
      kv("angular_z_rad_s", number(velocity.angular.z)), kv("check_xy_before", boolean(check_xy_before)),
      kv("check_xy_after", boolean(check_xy_after)), kv("base_result", boolean(result)),
    });
  } catch (...) {}
}

void TelemetryGoalChecker::publishToleranceTrace(
  const geometry_msgs::msg::Pose & pose_tolerance, bool result) noexcept
{
  try {publishTrace("get_tolerances", {
    kv("returned_pose_x_m", number(pose_tolerance.position.x)),
    kv("returned_pose_y_m", number(pose_tolerance.position.y)),
    kv("returned_pose_yaw_quaternion_z", number(pose_tolerance.orientation.z)),
    kv("returned_pose_yaw_quaternion_w", number(pose_tolerance.orientation.w)),
    kv("base_result", boolean(result)),
  });} catch (...) {}
}

void TelemetryGoalChecker::publishTrace(const char * event, std::vector<KeyValue> event_values) noexcept
{
  try {
    telemetryFaultPoint();
    if (!node_ || !trace_pub_) {return;}
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = node_->get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = plugin_name_; status.hardware_id = "factory_ops_goal_checker"; status.message = event;
    {
      std::lock_guard<std::mutex> lock(trace_mutex_);
      ++plugin_call_sequence_;
      status.values = {
        kv("schema_version", std::to_string(kTraceSchemaVersion)), kv("event", event),
        kv("event_sequence", std::to_string(nextEventSequence())),
        kv("plugin_call_sequence", std::to_string(plugin_call_sequence_)),
        kv("reset_epoch", std::to_string(reset_epoch_)), kv("plugin_name", plugin_name_),
        kv("action_xy_tolerance_m", number(xy_goal_tolerance_)),
        kv("action_yaw_tolerance_rad", number(yaw_goal_tolerance_)),
      };
    }
    status.values.insert(status.values.end(), event_values.begin(), event_values.end());
    message.status.push_back(std::move(status));
    trace_pub_->publish(message);
  } catch (...) {}
}

}  // namespace factory_ops_goal_checker

PLUGINLIB_EXPORT_CLASS(
  factory_ops_goal_checker::TelemetryGoalChecker,
  nav2_core::GoalChecker)
