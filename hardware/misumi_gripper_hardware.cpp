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
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  default_speed_percent_ = std::stoi(info_.hardware_parameters["default_speed_percent"]);
  default_torque_percent_ = std::stoi(info_.hardware_parameters["default_torque_percent"]);

  const auto num_joints = info_.joints.size();
  if (num_joints == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "No joints defined in hardware info.");
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
        RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Joint '%s' must have one 'position' command interface.", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 从每个关节的 parameters 中读取 slave_id
    try {
      slave_ids_[i] = std::stoi(joint.parameters.at("slave_id"));
      RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Joint '%s' mapped to slave_id %d", joint.name.c_str(), slave_ids_[i]);
    } catch (const std::out_of_range& ex) {
      RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Parameter 'slave_id' not found for joint '%s'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    } catch (const std::invalid_argument& ex) {
      RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Invalid 'slave_id' for joint '%s'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MisumiGripperHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Configuring Misumi Gripper Hardware...");
  // 1. 创建并连接共享的总线
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Connecting to bus on device '%s'...", device_.c_str());
  gripper_bus_ = std::make_unique<MisumiGripperBus>(device_, baud_rate_);
  if (!gripper_bus_->connect()) {
    RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Failed to connect to gripper bus: %s", gripper_bus_->getLastError().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Bus connected successfully.");

  // --- MODIFICATION START ---
  // 2. 为每个 slave_id 创建一个 MisumiGripper 客户端
  gripper_clients_.clear();
  for (int slave_id : slave_ids_) {
    gripper_clients_.push_back(std::make_unique<MisumiGripper>(*gripper_bus_, slave_id));
  }
  // --- MODIFICATION END ---
  
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Created %zu gripper clients.", gripper_clients_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MisumiGripperHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Cleaning up Misumi Gripper Hardware...");
  // 首先清理客户端，然后断开总线连接
  gripper_clients_.clear();
  if (gripper_bus_) {
    gripper_bus_->disconnect();
    gripper_bus_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MisumiGripperHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Activating Misumi Gripper Hardware...");
  // 激活所有夹爪
  for (size_t i = 0; i < gripper_clients_.size(); ++i) {
  if (!gripper_clients_[i]->enable()) {
      RCLCPP_ERROR(rclcpp::get_logger("MisumiGripperHardware"), "Failed to enable gripper with slave_id %d: %s",
                   slave_ids_[i], gripper_clients_[i]->getLastError().c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Gripper with slave_id %d enabled.", slave_ids_[i]);
  }

  // 初始化状态和命令
  std::fill(hw_states_position_.begin(), hw_states_position_.end(), 0.0);
  std::fill(hw_states_velocity_.begin(), hw_states_velocity_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), std::numeric_limits<double>::quiet_NaN());
  
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "All grippers activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MisumiGripperHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MisumiGripperHardware"), "Deactivating Misumi Gripper Hardware...");
  // 禁用所有夹爪
  for (size_t i = 0; i < gripper_clients_.size(); ++i) {
    if (!gripper_clients_[i]->disable()) {
      RCLCPP_WARN(rclcpp::get_logger("MisumiGripperHardware"), "Failed to disable gripper with slave_id %d: %s",
                  slave_ids_[i], gripper_clients_[i]->getLastError().c_str());
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MisumiGripperHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MisumiGripperHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type MisumiGripperHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // --- MODIFICATION START ---
  // 遍历每个关节/客户端，并读取其状态
  for (size_t i = 0; i < gripper_clients_.size(); ++i) {
    GripperStatus status;
    if (gripper_clients_[i]->readStatus(status)) {
      hw_states_position_[i] = status.position_mm / 1000.0; // mm to m
      hw_states_velocity_[i] = 0.0; // 速度百分比不是物理速度，设为0或估算
    } else {
      RCLCPP_WARN(rclcpp::get_logger("MisumiGripperHardware"), "Failed to read status from slave_id %d: %s",
                  slave_ids_[i], gripper_clients_[i]->getLastError().c_str());
    }
  }
  // --- MODIFICATION END ---
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MisumiGripperHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 遍历每个关节的命令，并发送到对应的客户端
  for (size_t i = 0; i < gripper_clients_.size(); ++i) {
    if (!std::isnan(hw_commands_[i])) {
      double position_mm = hw_commands_[i] * 1000.0; // m to mm
      
      if (!gripper_clients_[i]->moveTo(position_mm, default_speed_percent_, default_torque_percent_)) {
         RCLCPP_WARN(rclcpp::get_logger("MisumiGripperHardware"), "Failed to move gripper for slave_id %d: %s",
                     slave_ids_[i], gripper_clients_[i]->getLastError().c_str());
      }
      
      // 重置该关节的命令
      hw_commands_[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace misumi_gripper_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  supre_robot_control::MisumiGripperHardware,
  hardware_interface::SystemInterface)
