# supre_robot_control

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