# test_arm.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action.client import ClientGoalHandle, GoalStatus

class TrajectoryActionClient(Node):
    """
    一个可复用的 Action Client 类，用于发送轨迹目标并等待其完成。
    这个类本身不需要修改，因为它已经很好地封装了单个动作的执行逻辑。
    """
    def __init__(self):
        super().__init__('trajectory_action_client')
        action_qos_profile = rclpy.qos.qos_profile_system_default
        # 确认控制器名称与YAML文件一致
        self._action_client = ActionClient(self, 
                                           FollowJointTrajectory, 
                                           '/supre_robot_follower/right_arm_trajectory_controller/follow_joint_trajectory',
                                           goal_service_qos_profile=action_qos_profile,
                                           result_service_qos_profile=action_qos_profile,
                                           cancel_service_qos_profile=action_qos_profile,
                                           feedback_topic_qos_profile=action_qos_profile,
                                           status_topic_qos_profile=action_qos_profile)

    def send_goal(self, positions, time_from_start_sec=2):
        """
        发送一个目标轨迹点，并阻塞程序直到该动作完成。
        
        :param positions: 6个关节的目标位置列表 (list of float)
        :param time_from_start_sec: 到达目标位置所需的时间 (int)
        :return: True 如果动作成功, False 如果失败.
        """
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        # 确保关节名称顺序与控制器配置一致
        trajectory.joint_names = [
            'follower_right_arm_joint_1', 'follower_right_arm_joint_2', 'follower_right_arm_joint_3',
            'follower_right_arm_joint_4', 'follower_right_arm_joint_5', 'follower_right_arm_joint_6',
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions] # 确保是浮点数
        point.time_from_start = Duration(sec=time_from_start_sec, nanosec=0)
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self.get_logger().info('等待 Action Server 连接...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'发送目标 {positions} ...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)

        self.get_logger().info('等待 goal 被接受...')
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle: ClientGoalHandle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal 被服务器拒绝!')
            return False

        self.get_logger().info('Goal 被服务器接受! 等待执行完成...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result_status = get_result_future.result().status
        if result_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Action 执行成功!')
            return True
        else:
            self.get_logger().error(f'Action 执行失败，状态码: {result_status}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    action_client = TrajectoryActionClient()

    # 定义你想要循环执行的动作序列
    sequence_of_goals = [
        {'positions': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'time_sec': 4},
        {'positions': [0.0, 0.0, 0.0, 90.0, 0.0, 0.0], 'time_sec': 4},
        {'positions': [0.0, 90.0, 0.0, 90.0, 0.0, 0.0], 'time_sec': 3},
        {'positions': [0.0, 0.0, 0.0, 90.0, 0.0, 0.0], 'time_sec': 5},
    ]

    loop_count = 0
    # 使用 try...except...finally 结构确保即使出错或被中断，资源也能被正确清理
    try:
        # ===============================================================
        # 添加外部循环，只要 ROS 节点在运行，就一直循环
        # ===============================================================
        while rclpy.ok():
            loop_count += 1
            action_client.get_logger().info(f"========= 开始第 {loop_count} 轮动作序列 =========")

            # 内部循环，执行单个序列中的所有动作
            for i, goal in enumerate(sequence_of_goals):
                # 在开始每个新动作前，再次检查节点是否仍在运行
                if not rclpy.ok():
                    action_client.get_logger().info("接收到关闭信号，中断动作序列。")
                    break

                action_client.get_logger().info(f"--- 正在执行序列中的第 {i + 1}/{len(sequence_of_goals)} 个目标 ---")
                success = action_client.send_goal(goal['positions'], goal['time_sec'])
                
                if not success:
                    action_client.get_logger().error(f"目标 {i + 1} 执行失败，本轮序列中止。")
                    # 跳出内部的 for 循环，但外部的 while 循环会继续，开始新一轮
                    break
            else:
                # 如果 for 循环正常完成（没有被 break），则执行这里
                action_client.get_logger().info(f"第 {loop_count} 轮动作序列成功完成！")

    except KeyboardInterrupt:
        action_client.get_logger().info('程序被手动中断 (Ctrl+C)')
    finally:
        # 清理资源
        action_client.get_logger().info('正在关闭节点并清理资源...')
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()