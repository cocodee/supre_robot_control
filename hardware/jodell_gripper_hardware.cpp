#include "supre_robot_control/jodell_gripper_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include <cmath>
namespace supre_robot_control
{

uint8_t convertToGripperPosition(double position) {
  return static_cast<uint8_t>(position * 255.0);
}

double convertFromGripperPosition(uint8_t position) {
  return static_cast<double>(position) / 255.0;
}

uint8_t convertToGripperPercentage(int percentage) {
  return static_cast<uint8_t>(percentage * 255/100);
}

int convertFromGripperPercentage(uint8_t percentage) {
  return static_cast<int>(percentage * 100/255);
}
hardware_interface::CallbackReturn JodellGripperHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get hardware parameters
  device_ = info_.hardware_parameters["device"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  default_speed_percent_ = std::stoi(info_.hardware_parameters["default_speed_percent"]);
  default_torque_percent_ = std::stoi(info_.hardware_parameters["default_torque_percent"]);

  const auto num_joints = info_.joints.size();
  if (num_joints == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("JodellGripperHardware"), "No joints defined in hardware info.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // 根据关节数量调整向量大小
  slave_ids_.resize(num_joints);
  hw_commands_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_states_position_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());

  // --- MODIFICATION START ---
  // 遍历所有关节，验证接口并读取每个关节的 slave_id
  for (size_t i = 0; i < num_joints; ++i)
  {
    const auto& joint = info_.joints[i];
    // 验证接口 (此处简化，可添加之前的详细验证)
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_ERROR(rclcpp::get_logger("JodellGripperHardware"), "Joint '%s' must have one 'position' command interface.", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 从每个关节的 parameters 中读取 slave_id
    try {
      slave_ids_[i] = std::stoi(joint.parameters.at("slave_id"));
      RCLCPP_INFO(rclcpp::get_logger("JodellripperHardware"), "Joint '%s' mapped to slave_id %d", joint.name.c_str(), slave_ids_[i]);
    } catch (const std::out_of_range& ex) {
      RCLCPP_ERROR(rclcpp::get_logger("JodellGripperHardware"), "Parameter 'slave_id' not found for joint '%s'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    } catch (const std::invalid_argument& ex) {
      RCLCPP_ERROR(rclcpp::get_logger("JodellGripperHardware"), "Invalid 'slave_id' for joint '%s'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JodellGripperHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JodellGripperHardware"), "Configuring Jodell Gripper Hardware...");
  // 1. 创建并连接共享的总线
  RCLCPP_INFO(rclcpp::get_logger("JodellGripperHardware"), "Connecting to bus on device '%s'...", device_.c_str());
  gripper_bus_ = std::make_unique<GripperBus>(device_, baud_rate_);
  if (!gripper_bus_->connect()) {
    RCLCPP_ERROR(rclcpp::get_logger("JodellGripperHardware"), "Failed to connect to gripper bus: %s", gripper_bus_->getLastError().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("JodellGripperHardware"), "Bus connected successfully.");

  // --- MODIFICATION START ---
  // 2. 为每个 slave_id 创建一个 JodellGripper 客户端
  gripper_clients_.clear();
  for (int slave_id : slave_ids_) {
    gripper_clients_.push_back(std::make_unique<JodellGripper>(*gripper_bus_, slave_id));
  }
  // --- MODIFICATION END ---
  
  RCLCPP_INFO(rclcpp::get_logger("JodellGripperHardware"), "Created %zu gripper clients.", gripper_clients_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JodellGripperHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JodellGripperHardware"), "Cleaning up Jodell Gripper Hardware...");
  // 首先清理客户端，然后断开总线连接
  gripper_clients_.clear();
  if (gripper_bus_) {
    gripper_bus_->disconnect();
    gripper_bus_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JodellGripperHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JodellGripperHardware"), "Activating Jodell Gripper Hardware...");
  // 激活所有夹爪
  for (size_t i = 0; i < gripper_clients_.size(); ++i) {
  if (!gripper_clients_[i]->enable()) {
      RCLCPP_ERROR(rclcpp::get_logger("JodellGripperHardware"), "Failed to enable gripper with slave_id %d: %s",
                   slave_ids_[i], "");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("JodellGripperHardware"), "Gripper with slave_id %d enabled.", slave_ids_[i]);
  }

  // 初始化状态和命令
  std::fill(hw_states_position_.begin(), hw_states_position_.end(), 0.0);
  std::fill(hw_states_velocity_.begin(), hw_states_velocity_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), std::numeric_limits<double>::quiet_NaN());
  
  RCLCPP_INFO(rclcpp::get_logger("JodellGripperHardware"), "All grippers activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JodellGripperHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JodellGripperHardware"), "Deactivating Jodell Gripper Hardware...");
  // 禁用所有夹爪
  for (size_t i = 0; i < gripper_clients_.size(); ++i) {
    if (!gripper_clients_[i]->disable()) {
      RCLCPP_WARN(rclcpp::get_logger("JodellGripperHardware"), "Failed to disable gripper with slave_id %d: %s",
                  slave_ids_[i], "".c_str());
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JodellGripperHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JodellGripperHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type JodellGripperHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // --- MODIFICATION START ---
  // 遍历每个关节/客户端，并读取其状态
  for (size_t i = 0; i < gripper_clients_.size(); ++i) {
    GripperStatus status;
    if (gripper_clients_[i]->getStatus(status)) {
      hw_states_position_[i] = convertFromGripperPosition(status.position); // mm to m
      hw_states_velocity_[i] = 0.0; // 速度百分比不是物理速度，设为0或估算
    } else {
      RCLCPP_WARN(rclcpp::get_logger("JodellGripperHardware"), "Failed to read status from slave_id %d: %s",
                  slave_ids_[i], "");
    }
  }
  // --- MODIFICATION END ---
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JodellGripperHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 遍历每个关节的命令，并发送到对应的客户端
  for (size_t i = 0; i < gripper_clients_.size(); ++i) {
    if (!std::isnan(hw_commands_[i])) {
      uint8_t position = convertToGripperPosition(hw_commands_[i]);
      uint8_t speed_percent = convertToGripperPercentage(default_speed_percent_);
      uint8_t torque_percent = convertToGripperPercentage(default_torque_percent_);
      if (!gripper_clients_[i]->move(position, speed_percent, torque_percent)) {
         RCLCPP_WARN(rclcpp::get_logger("JodellGripperHardware"), "Failed to move gripper for slave_id %d: %s",
                     slave_ids_[i], "");
      }
      
      // 重置该关节的命令
      hw_commands_[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace Jodell_gripper_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  supre_robot_control::JodellGripperHardware,
  hardware_interface::SystemInterface)
