#ifndef FACTORY_OPS_GOAL_CHECKER__TELEMETRY_REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define FACTORY_OPS_GOAL_CHECKER__TELEMETRY_REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

namespace factory_ops_goal_checker
{

class TelemetryRegulatedPurePursuitController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  static constexpr int kTraceSchemaVersion = 1;
  static constexpr const char * kTraceTopic = "controller_cycle_trace";

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void reset() override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  virtual void telemetryFaultPoint() {}
  virtual bool prepareGoalChecker(nav2_core::GoalChecker *) {return false;}

private:
  void publishComputeTrace(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    bool rotating_before, bool reached_before,
    const geometry_msgs::msg::TwistStamped * output,
    const char * phase, const char * command_reason,
    const char * error_type, const char * error_message) noexcept;

  rclcpp_lifecycle::LifecycleNode::SharedPtr trace_node_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr trace_pub_;
  std::uint64_t plugin_call_sequence_{0};
  std::uint64_t plan_epoch_{0};
  std::uint64_t reset_epoch_{0};
};

}  // namespace factory_ops_goal_checker

#endif  // FACTORY_OPS_GOAL_CHECKER__TELEMETRY_REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
