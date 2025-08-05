#include "supre_robot_control/misumi_gripper_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include <cmath>
namespace supre_robot_control
{

hardware_interface::CallbackReturn MisumiGripperHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get hardware parameters
  device_ = info_.hardware_parameters["device"];
  slave_id_ = std::stoi(info_.hardware_parameters["slave_id"]);
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  default_speed_percent_ = std::stoi(info_.hardware_parameters["default_speed_percent"]);
  default_torque_percent_ = std::stoi(info_.hardware_parameters["default_torque_percent"]);

  // Check joints
  if (info_.joints.size() != 1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Expect 1 joint, but got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check command interfaces
  if (info_.joints[0].command_interfaces.size() != 1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Expect 1 command interface, but got %zu", info_.joints[0].command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints[0].command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Expect 'position' command interface, but got '%s'", info_.joints[0].command_interfaces[0].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check state interfaces
  if (info_.joints[0].state_interfaces.size() != 1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Expect 1 state interface, but got %zu", info_.joints[0].state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints[0].state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Expect 'position' state interface, but got '%s'", info_.joints[0].state_interfaces[0].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MisumiGripperHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Configuring Misumi Gripper Hardware...");
  gripper_client_ = std::make_unique<MisumiGripper>(device_, slave_id_, baud_rate_);
  if (!gripper_client_->connect())
  {
    RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Failed to connect to gripper: %s", gripper_client_->getLastError().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Successfully connected to gripper.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MisumiGripperHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Cleaning up Misumi Gripper Hardware...");
  gripper_client_->disconnect();
  gripper_client_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MisumiGripperHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Activating Misumi Gripper Hardware...");
  if (!gripper_client_->enable())
  {
      RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Failed to enable gripper: %s", gripper_client_->getLastError().c_str());
      return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Gripper enabled.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MisumiGripperHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Deactivating Misumi Gripper Hardware...");
  if (gripper_client_){
      if (!gripper_client_->disable())
      {
          RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Failed to disable gripper: %s", gripper_client_->getLastError().c_str());
          return hardware_interface::CallbackReturn::ERROR;
      }
  }

  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Gripper disabled.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MisumiGripperHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[0]));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MisumiGripperHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_commands_[0]));
  return command_interfaces;
}

hardware_interface::return_type MisumiGripperHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  GripperStatus status;
  if (gripper_client_->readStatus(status))
  {
    // The position unit in your class is mm, but ros2_control standard is meters.
    hw_states_position_[0] = status.position_mm / 1000.0; 
    hw_states_velocity_[0] = status.speed_percent;
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("MisumiGripperHardware"), "Failed to read gripper status: %s", gripper_client_->getLastError().c_str());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MisumiGripperHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!std::isnan(hw_commands_[0]))
  {
    // The position unit in ros2_control is meters, convert to mm for your class.
    double position_mm = hw_commands_[0] * 1000.0;
    if (!gripper_client_->moveTo(position_mm, default_speed_percent_, default_torque_percent_))
    {
       RCLCPP_WARN(rclcpp::get_logger("MisumiGripperHardware"), "Failed to move gripper: %s", gripper_client_->getLastError().c_str());
    }
    // Reset command to NaN after execution
    hw_commands_[0] = std::numeric_limits<double>::quiet_NaN();
  }
  return hardware_interface::return_type::OK;
}

}  // namespace misumi_gripper_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  supre_robot_control::MisumiGripperHardware,
  hardware_interface::SystemInterface)
