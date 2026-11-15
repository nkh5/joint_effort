#include <chrono>
#include <csignal>
#include <memory>
#include <moveit_servo/servo_node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
volatile std::sig_atomic_t stopping = 0;
void request_stop(int)
{
  stopping = 1;
}
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  {
    auto servo = std::make_shared<moveit_servo::ServoNode>(rclcpp::NodeOptions());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(servo->get_node_base_interface());
    while (!stopping && rclcpp::ok())
      executor.spin_once(std::chrono::milliseconds(20));
    executor.remove_node(servo->get_node_base_interface());
    // Let Servo's worker threads finish while the ROS context is still valid.
    servo.reset();
  }
  rclcpp::shutdown();
  return 0;
}
