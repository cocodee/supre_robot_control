# supre_robot_control

  项目名称：`supre_robot_control`

  项目简介

  supre_robot_control 是一个基于 ROS2 (Robot Operating System 2)
  的机器人控制项目，旨在实现对机械臂的精确控制。从项目结构和启动脚本来看，该项目核心功能是实现一个"领导者-跟随者"
  (Leader-Follower)模式的双臂协作系统。

  此系统通常包含两台机械臂：一台作为“领导者”，由操作员手动控制或通过输入设备引导；另一台作为“跟随者”，实时模仿“领导者”的动作。这种
  模式在远程操作、复杂任务示教和自动化任务执行中非常有用。

  项目环境依赖于 Conda 提供的 ros2_env 环境，并在启动前配置硬件串口权限 (/dev/ttyTHS1)，表明它需要与具体的硬件设备进行底层通信。

  核心启动脚本功能介绍

  ##### 1. start_common_gripper_leader_follower.sh

  这个脚本用于启动一个带夹爪控制的领导者-跟随者模式。

   * 核心功能:
       * 它会启动一个ROS2节点，该节点不仅同步领导者和跟随者机械臂的本体运动，还同步控制它们末端夹爪（Gripper）的开合状态。
       * 当操作员控制领导者机械臂并操作其夹爪时，跟随者机械臂会精确地模仿所有这些动作，包括手臂的姿态和夹爪的操作。
   * 应用场景:
       * 需要进行远程抓取、放置、装配等需要手眼协调和物体操作的任务。
       * 通过示教方式，让机器人学习并重复一套复杂的抓取操作流程。

  ##### 2. start_common_follower_trajectory.sh

  这个脚本用于启动跟随者机械臂的轨迹复现模式。

   * 核心功能:
       * 它启动一个独立的ROS2节点，让跟随者机械臂自主执行一个预先录制或生成的运动轨迹。
       * 在此模式下，跟随者机械臂的运动不再依赖于领导者机械臂的实时输入，而是根据程序化的路径点（Trajectory）精确移动。
   * 应用场景:
       * 自动化生产线上的重复性任务，如焊接、喷涂、码垛等。
       * 在领导者-跟随者模式下录制好一段任务轨迹后，使用此脚本让机器人进行离线、自主、重复地执行。

## 控制器 (Controllers)
通过 `ros2 control list_controllers` 命令可以查看到以下活动的控制器：

| 控制器名称 (Controller Name) | 控制器类型 (Controller Type) | 状态 (State) |
| :--- | :--- | :--- |
| `misumi_gripper_controller` | `forward_command_controller/ForwardCommandController` | `active` |
| `left_arm_controller` | `position_controllers/JointGroupPositionController` | `active` |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | `active` |
| `right_arm_controller` | `position_controllers/JointGroupPositionController` | `active` |

## 话题 (Topics)
通过 `ros2 topic list` 命令可以查看到以下ROS话题：

#### 控制器相关话题
*   **左臂控制器 (`left_arm_controller`)**:
    *   `/left_arm_controller/commands`: (类型: `std_msgs/msg/Float64MultiArray`) 用于向左臂发送目标位置指令。
    *   `/left_strom_controller/transition_event`: (类型: `lifecycle_msgs/msg/TransitionEvent`) 发布控制器生命周期状态变化的事件。
*   **右臂控制器 (`right_arm_controller`)**:
    *   `/right_arm_controller/commands`: (类型: `std_msgs/msg/Float64MultiArray`) 用于向右臂发送目标位置指令。
    *   `/right_arm_controller/transition_event`: (类型: `lifecycle_msgs/msg/TransitionEvent`) 发布控制器生命周期状态变化的事件。
*   **夹爪控制器 (`misumi_gripper_controller`)**:
    *   `/misumi_gripper_controller/commands`: (类型: `std_msgs/msg/Float64MultiArray`) 用于向夹爪发送目标位置指令。
    *   `/misumi_gripper_controller/transition_event`: (类型: `lifecycle_msgs/msg/TransitionEvent`) 发布控制器生命周期状态变化的事件。
*   **关节状态广播器 (`joint_state_broadcaster`)**:
    *   `/joint_state_broadcaster/transition_event`: (类型: `lifecycle_msgs/msg/TransitionEvent`) 发布广播器生命周期状态变化的事件。

#### 机器人状态话题
*   `/joint_states`: (类型: `sensor_msgs/msg/JointState`) 由 `joint_state_broadcaster` 发布，包含所有受控关节的当前状态（位置、速度、力矩）。
*   `/dynamic_joint_states`: (类型: `control_msgs/msg/DynamicJointState`) 由 `joint_state_broadcaster` 发布，包含关节状态以及所有可用接口的值。
*   `/robot_description`: (类型: `std_msgs/msg/String`) 包含机器人的URDF模型。
*   `/tf`: (类型: `tf2_msgs/msg/TFMessage`) 发布动态坐标系变换。
*   `/tf_static`: (类型: `tf2_msgs/msg/TFMessage`) 发布静态坐标系变换。

#### 系统话题
*   `/parameter_events`: ROS 2参数系统的事件通知。
*   `/rosout`: ROS 2的日志消息。