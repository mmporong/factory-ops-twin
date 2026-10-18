#include "factory_ops_goal_checker/confirmed_xy_regulated_pure_pursuit_controller.hpp"

#include <stdexcept>

#include "factory_ops_goal_checker/telemetry_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace factory_ops_goal_checker
{

void ConfirmedXyRegulatedPurePursuitController::prepareGoalChecker(
  nav2_core::GoalChecker * goal_checker)
{
  auto * confirmed_goal_checker = dynamic_cast<TelemetryGoalChecker *>(goal_checker);
  if (confirmed_goal_checker == nullptr) {
    throw std::invalid_argument(
            "ConfirmedXyRegulatedPurePursuitController requires TelemetryGoalChecker");
  }
  if (!confirmed_goal_checker->xyConfirmed()) {
    has_reached_xy_tolerance_ = false;
  }
}

}  // namespace factory_ops_goal_checker

PLUGINLIB_EXPORT_CLASS(
  factory_ops_goal_checker::ConfirmedXyRegulatedPurePursuitController,
  nav2_core::Controller)
