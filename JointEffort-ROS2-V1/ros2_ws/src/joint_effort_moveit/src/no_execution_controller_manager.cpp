// SPDX-License-Identifier: BSD-3-Clause
// MoveItCpp creates a trajectory-execution manager even when move_group path
// execution is disabled. This explicit empty manager prevents a misleading
// missing-plugin fatal message and never exposes a path execution controller.
#include <moveit/controller_manager/controller_manager.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace joint_effort_moveit
{
class NoExecutionControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  void initialize(const rclcpp::Node::SharedPtr&) override {}
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string&) override
  { return nullptr; }
  void getControllersList(std::vector<std::string>& names) override { names.clear(); }
  void getActiveControllers(std::vector<std::string>& names) override { names.clear(); }
  void getControllerJoints(const std::string&, std::vector<std::string>& joints) override { joints.clear(); }
  ControllerState getControllerState(const std::string&) override { return {}; }
  bool switchControllers(const std::vector<std::string>&, const std::vector<std::string>&) override
  { return false; }
};
}  // namespace joint_effort_moveit
PLUGINLIB_EXPORT_CLASS(joint_effort_moveit::NoExecutionControllerManager,
                      moveit_controller_manager::MoveItControllerManager)
