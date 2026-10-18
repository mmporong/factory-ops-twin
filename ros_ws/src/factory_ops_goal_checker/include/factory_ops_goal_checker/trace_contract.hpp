#ifndef FACTORY_OPS_GOAL_CHECKER__TRACE_CONTRACT_HPP_
#define FACTORY_OPS_GOAL_CHECKER__TRACE_CONTRACT_HPP_

#include <atomic>
#include <cstdint>

#include "rclcpp/qos.hpp"

namespace factory_ops_goal_checker
{

inline std::uint64_t nextEventSequence()
{
  static std::atomic<std::uint64_t> sequence{0};
  return sequence.fetch_add(1, std::memory_order_relaxed) + 1;
}

inline rclcpp::QoS traceQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(1024)).reliable().transient_local();
}

}  // namespace factory_ops_goal_checker

#endif  // FACTORY_OPS_GOAL_CHECKER__TRACE_CONTRACT_HPP_
