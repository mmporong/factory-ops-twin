#include "factory_ops_goal_checker/telemetry_regulated_pure_pursuit_controller.hpp"

#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string_view>
#include <typeinfo>
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

std::string boolean(bool value) {return value ? "true" : "false";}

double yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

void TelemetryRegulatedPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::configure(
    parent, name, std::move(tf), std::move(costmap_ros));
  try {
    trace_node_ = parent.lock();
    if (trace_node_) {
      trace_pub_ = trace_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(kTraceTopic, traceQoS());
    }
  } catch (...) {trace_pub_.reset();}
}

void TelemetryRegulatedPurePursuitController::activate()
{
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::activate();
  try {if (trace_pub_) {trace_pub_->on_activate();}} catch (...) {}
}

void TelemetryRegulatedPurePursuitController::deactivate()
{
  try {if (trace_pub_) {trace_pub_->on_deactivate();}} catch (...) {}
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::deactivate();
}

void TelemetryRegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::setPlan(path);
  ++plan_epoch_;
}

void TelemetryRegulatedPurePursuitController::reset()
{
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::reset();
  ++reset_epoch_;
}

geometry_msgs::msg::TwistStamped TelemetryRegulatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const bool rotating_before = is_rotating_to_heading_;
  const bool reached_before = has_reached_xy_tolerance_;
  ++plugin_call_sequence_;
  publishComputeTrace(pose, velocity, rotating_before, reached_before, nullptr, "begin", "", "");
  try {
    prepareGoalChecker(goal_checker);
    auto output = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::
      computeVelocityCommands(pose, velocity, goal_checker);
    publishComputeTrace(pose, velocity, rotating_before, reached_before, &output, "end", "", "");
    return output;
  } catch (const std::exception & error) {
    publishComputeTrace(pose, velocity, rotating_before, reached_before, nullptr, "end",
      typeid(error).name(), error.what());
    throw;
  } catch (...) {
    publishComputeTrace(pose, velocity, rotating_before, reached_before, nullptr, "end",
      "unknown", "non-standard exception");
    throw;
  }
}

void TelemetryRegulatedPurePursuitController::publishComputeTrace(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  bool rotating_before, bool reached_before,
  const geometry_msgs::msg::TwistStamped * output,
  const char * phase, const char * error_type, const char * error_message) noexcept
{
  try {
  telemetryFaultPoint();
  if (!trace_node_ || !trace_pub_) {return;}
  const std::string_view phase_value{phase};
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = trace_node_->get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = (phase_value == "begin" || output) ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.name = plugin_name_;
  status.hardware_id = "factory_ops_goal_checker";
  status.message = "compute_velocity_commands";
  const auto incoming_stamp_ns = static_cast<std::int64_t>(pose.header.stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(pose.header.stamp.nanosec);
  status.values = {
    kv("schema_version", std::to_string(kTraceSchemaVersion)),
    kv("event", "compute_velocity_commands"),
    kv("event_sequence", std::to_string(nextEventSequence())),
    kv("plugin_call_sequence", std::to_string(plugin_call_sequence_)),
    kv("plugin_name", plugin_name_),
    kv("phase", std::string(phase_value)),
    kv("plan_epoch", std::to_string(plan_epoch_)),
    kv("reset_epoch", std::to_string(reset_epoch_)),
    kv("incoming_pose_stamp_ns", std::to_string(incoming_stamp_ns)),
    kv("incoming_pose_frame_id", pose.header.frame_id),
    kv("pose_x_m", number(pose.pose.position.x)),
    kv("pose_y_m", number(pose.pose.position.y)),
    kv("pose_yaw_rad", number(yaw(pose.pose.orientation))),
    kv("velocity_linear_x_m_s", number(velocity.linear.x)),
    kv("velocity_linear_y_m_s", number(velocity.linear.y)),
    kv("velocity_angular_z_rad_s", number(velocity.angular.z)),
    kv("goal_dist_tol_m", number(goal_dist_tol_)),
    kv("is_rotating_before", boolean(rotating_before)),
    kv("is_rotating_after", boolean(is_rotating_to_heading_)),
    kv("has_reached_xy_before", boolean(reached_before)),
    kv("has_reached_xy_after", boolean(has_reached_xy_tolerance_)),
    kv("outcome", phase_value == "begin" ? "pending" : output ? "returned" : "threw"),
    kv("output_linear_x_m_s", output ? number(output->twist.linear.x) : ""),
    kv("output_linear_y_m_s", output ? number(output->twist.linear.y) : ""),
    kv("output_angular_z_rad_s", output ? number(output->twist.angular.z) : ""),
    kv("error_type", error_type),
    kv("error_message", error_message),
  };
  message.status.push_back(std::move(status));
  trace_pub_->publish(message);
  } catch (...) {}
}

}  // namespace factory_ops_goal_checker

PLUGINLIB_EXPORT_CLASS(
  factory_ops_goal_checker::TelemetryRegulatedPurePursuitController,
  nav2_core::Controller)
