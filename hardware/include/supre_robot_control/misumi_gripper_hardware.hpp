#ifndef MISUMI_GRIPPER_HARDWARE_HPP_
#define MISUMI_GRIPPER_HARDWARE_HPP_

#include <vector>
#include <string>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "MisumiGripper.hpp"

namespace supre_robot_control
{
class MisumiGripperHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MisumiGripperHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- MODIFICATION START ---
  // 单一的、共享的通信总线
  std::unique_ptr<MisumiGripperBus> gripper_bus_;
  
  // 每个关节对应一个夹爪客户端实例
  std::vector<std::unique_ptr<MisumiGripper>> gripper_clients_;
  
  // 存储每个关节对应的 slave_id
  std::vector<int> slave_ids_;
  // --- MODIFICATION END ---  
  // Store the commands for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  // Parameters
  // Store hardware parameters
  std::string device_;
  int slave_id_;
  int baud_rate_;
  int default_speed_percent_;
  int default_torque_percent_;
};

}  // namespace misumi_gripper_hardware

#endif  // MISUMI_GRIPPER_HARDWARE_HPP_
