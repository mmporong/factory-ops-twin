#ifndef FACTORY_OPS_GOAL_CHECKER__CONFIRMED_XY_REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define FACTORY_OPS_GOAL_CHECKER__CONFIRMED_XY_REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include "factory_ops_goal_checker/telemetry_regulated_pure_pursuit_controller.hpp"

namespace factory_ops_goal_checker
{

class ConfirmedXyRegulatedPurePursuitController
  : public TelemetryRegulatedPurePursuitController
{
public:
  static constexpr const char * kBehaviorMode = "goal_checker_confirmed_xy_latch";

protected:
  void prepareGoalChecker(nav2_core::GoalChecker * goal_checker) override;
};

}  // namespace factory_ops_goal_checker

#endif  // FACTORY_OPS_GOAL_CHECKER__CONFIRMED_XY_REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
