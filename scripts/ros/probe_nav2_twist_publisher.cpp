// Isolated data fixture. Zero differential-drive velocity; unique probe namespace only.
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/twist_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  if (argc != 5) {return 2;}
  const std::string robot = argv[1];
  const int index = std::stoi(robot);
  const std::string stage = argv[2];
  const std::string prefix = argv[3];
  const double duration = std::stod(argv[4]);
  if (index < 1 || index > 4 || duration <= 0 || duration > 20 ||
    prefix.rfind("/probe_", 0) != 0 || prefix.size() != 39) {return 2;}
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<nav2_util::LifecycleNode>(
    "nav2_twist_fixture", prefix + "/to_windows/carter" + robot);
  auto publisher = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel", rclcpp::QoS(10));
  publisher->on_activate();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  using Clock = std::chrono::steady_clock;
  const auto deadline = Clock::now() + std::chrono::seconds(110);
  while (!std::filesystem::exists(stage + "/start.json")) {
    executor.spin_some();
    if (Clock::now() > deadline) {rclcpp::shutdown(); return 3;}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const auto end = Clock::now() + std::chrono::duration<double>(duration);
  int sent = 0;
  const auto first_matched = publisher->get_subscription_count();
  while (rclcpp::ok() && Clock::now() < end) {
    auto value = std::make_unique<geometry_msgs::msg::TwistStamped>();
    value->twist.linear.y = static_cast<double>(index);
    value->twist.linear.z = static_cast<double>(++sent);
    value->twist.angular.x = 12345.;
    publisher->publish(std::move(value));
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const auto last_matched = publisher->get_subscription_count();
  publisher->on_deactivate();
  std::ofstream out(stage + "/cpp-carter" + robot + ".json");
  out << "{\"robot\":" << index << ",\"sent\":" << sent <<
    ",\"matched_at_start\":" << first_matched << ",\"matched_at_end\":" << last_matched <<
    ",\"enable_stamped_cmd_vel\":false,\"linear_x\":0,\"angular_z\":0}\n";
  out.close();
  executor.remove_node(node->get_node_base_interface());
  publisher.reset(); node.reset(); rclcpp::shutdown();
  return 0;
}
