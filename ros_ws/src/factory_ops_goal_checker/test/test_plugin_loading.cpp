#include <memory>
#include <string>

#include "class_loader/class_loader.hpp"
#include "gtest/gtest.h"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"

TEST(FactoryOpsGoalCheckerPluginLoading, LoadsAllClassesFromOneSharedLibrary)
{
  class_loader::ClassLoader loader(FACTORY_OPS_GOAL_CHECKER_LIBRARY);
  auto goal_checker = loader.createInstance<nav2_core::GoalChecker>(
    "factory_ops_goal_checker::TelemetryGoalChecker");
  auto controller = loader.createInstance<nav2_core::Controller>(
    "factory_ops_goal_checker::TelemetryRegulatedPurePursuitController");
  auto confirmed_controller = loader.createInstance<nav2_core::Controller>(
    "factory_ops_goal_checker::ConfirmedXyRegulatedPurePursuitController");
  ASSERT_NE(nullptr, goal_checker);
  ASSERT_NE(nullptr, controller);
  ASSERT_NE(nullptr, confirmed_controller);
}
