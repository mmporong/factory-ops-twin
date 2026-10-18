#ifndef FACTORY_OPS_GOAL_CHECKER__INSET_TELEMETRY_GOAL_CHECKER_HPP_
#define FACTORY_OPS_GOAL_CHECKER__INSET_TELEMETRY_GOAL_CHECKER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace factory_ops_goal_checker
{

class TelemetryGoalChecker : public nav2_controller::SimpleGoalChecker
{
public:
  static constexpr int kTraceSchemaVersion = 1;
  static constexpr const char * kTraceTopic = "controller_cycle_trace";

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  using KeyValue = diagnostic_msgs::msg::KeyValue;

  diagnostic_msgs::msg::DiagnosticArray makeTrace(
    const std::string & event, const std::vector<KeyValue> & event_values);
  void publishTrace(const std::string & event, const std::vector<KeyValue> & event_values = {});
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr trace_pub_;
  std::mutex trace_mutex_;
  std::uint64_t plugin_call_sequence_{0};
  std::uint64_t reset_epoch_{0};
};

}  // namespace factory_ops_goal_checker

#endif  // FACTORY_OPS_GOAL_CHECKER__INSET_TELEMETRY_GOAL_CHECKER_HPP_
