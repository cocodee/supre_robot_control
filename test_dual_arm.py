# test_dual_arm.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action.client import ClientGoalHandle, GoalStatus

class DualArmTrajectoryExecutor(Node):
    """
    一个用于同时控制左右两个机械臂的节点。
    它会同时发送目标，并等待两个手臂都完成动作后才返回。
    """
    def __init__(self):
        super().__init__('dual_arm_trajectory_executor')
        action_qos_profile = rclpy.qos.qos_profile_system_default

        # 创建右臂的 Action Client
        self._right_arm_client = ActionClient(
            self, FollowJointTrajectory, 
            '/supre_robot_follower/right_arm_trajectory_controller/follow_joint_trajectory',
            goal_service_qos_profile=action_qos_profile,
            result_service_qos_profile=action_qos_profile,
            cancel_service_qos_profile=action_qos_profile)
            
        # 创建左臂的 Action Client
        self._left_arm_client = ActionClient(
            self, FollowJointTrajectory, 
            '/supre_robot_follower/left_arm_trajectory_controller/follow_joint_trajectory',
            goal_service_qos_profile=action_qos_profile,
            result_service_qos_profile=action_qos_profile,
            cancel_service_qos_profile=action_qos_profile)            
        
        self.get_logger().info('双臂轨迹执行器已启动。')
        
        # 定义关节名称 (假设左臂关节名称与右臂类似)
        self.right_joint_names = [
            'follower_right_arm_joint_1', 'follower_right_arm_joint_2', 'follower_right_arm_joint_3',
            'follower_right_arm_joint_4', 'follower_right_arm_joint_5', 'follower_right_arm_joint_6',
        ]
        self.left_joint_names = [
            'follower_left_arm_joint_1', 'follower_left_arm_joint_2', 'follower_left_arm_joint_3',
            'follower_left_arm_joint_4', 'follower_left_arm_joint_5', 'follower_left_arm_joint_6',
        ]

    def _create_goal_msg(self, joint_names, positions, time_from_start_sec):
        """辅助函数，用于创建 FollowJointTrajectory.Goal 消息。"""
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=time_from_start_sec, nanosec=0)
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        return goal_msg

    def send_dual_arm_goal_and_wait(self, left_positions, right_positions, time_sec):
        """
        异步发送目标到双臂，然后等待两者都完成。
        
        :return: True 如果双臂都成功, False 如果任何一个失败.
        """
        self.get_logger().info('等待左右臂的 Action Server...')
        self._left_arm_client.wait_for_server()
        self._right_arm_client.wait_for_server()
        self.get_logger().info('Action Server 已连接。')

        # 1. 创建左右臂的目标消息
        left_goal_msg = self._create_goal_msg(self.left_joint_names, left_positions, time_sec)
        right_goal_msg = self._create_goal_msg(self.right_joint_names, right_positions, time_sec)

        # 2. 异步发送目标，并获取 'send_goal_future'
        self.get_logger().info('正在异步发送目标到左右臂...')
        send_goal_future_left = self._left_arm_client.send_goal_async(left_goal_msg)
        send_goal_future_right = self._right_arm_client.send_goal_async(right_goal_msg)

        # 3. 等待服务器接受 Goal 的响应
        # 我们需要循环 spin 直到两个 future 都完成
        while rclpy.ok() and (not send_goal_future_left.done() or not send_goal_future_right.done()):
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle_left: ClientGoalHandle = send_goal_future_left.result()
        goal_handle_right: ClientGoalHandle = send_goal_future_right.result()
        
        # 4. 检查 Goal 是否被接受
        if not goal_handle_left.accepted or not goal_handle_right.accepted:
            if not goal_handle_left.accepted: self.get_logger().error('左臂目标被拒绝!')
            if not goal_handle_right.accepted: self.get_logger().error('右臂目标被拒绝!')
            return False
            
        self.get_logger().info('左右臂目标均被接受，等待执行完成...')

        # 5. 获取 'get_result_future'
        get_result_future_left = goal_handle_left.get_result_async()
        get_result_future_right = goal_handle_right.get_result_async()

        # 6. 等待两个动作都执行完成
        while rclpy.ok() and (not get_result_future_left.done() or not get_result_future_right.done()):
            rclpy.spin_once(self, timeout_sec=0.1)
            
        # 7. 检查最终结果
        status_left = get_result_future_left.result().status
        status_right = get_result_future_right.result().status
        
        success = True
        if status_left != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'左臂动作执行失败，状态码: {status_left}')
            success = False
        if status_right != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'右臂动作执行失败，状态码: {status_right}')
            success = False
        
        if success:
            self.get_logger().info('双臂动作均成功完成！')
        
        return success


def main(args=None):
    rclpy.init(args=args)
    
    executor = DualArmTrajectoryExecutor()

    # 定义双臂的动作序列
    # 每个元素包含左右臂的目标位置和运动时间
    sequence_of_goals = [
        {
            'left_positions':  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
            'right_positions': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
            'time_sec': 4
        },
        {
            'left_positions':  [0.0, 0.0, 0.0, -90.0, 0.0, 0.0], 
            'right_positions': [0.0, 0.0, 0.0, 90.0, 0.0, 0.0], 
            'time_sec': 3
        },
        {
            'left_positions':  [0.0, -90.0, 0.0, -90.0, 0.0, 0.0], 
            'right_positions': [0.0, 90.0, 0.0, 90.0, 0.0, 0.0], 
            'time_sec': 5
        },
        {
            'left_positions':  [0.0, 0.0, 0.0, -90.0, 0.0, 0.0], 
            'right_positions': [0.0, 0.0, 0.0, 90.0, 0.0, 0.0], 
            'time_sec': 3
        },        
    ]

    loop_count = 0
    try:
        while rclpy.ok():
            loop_count += 1
            executor.get_logger().info(f"========= 开始第 {loop_count} 轮双臂动作序列 =========")

            for i, goal in enumerate(sequence_of_goals):
                if not rclpy.ok(): break

                executor.get_logger().info(f"--- 正在执行序列中的第 {i + 1}/{len(sequence_of_goals)} 个目标 ---")
                
                # 调用我们的核心函数
                success = executor.send_dual_arm_goal_and_wait(
                    left_positions=goal['left_positions'],
                    right_positions=goal['right_positions'],
                    time_sec=goal['time_sec']
                )
                
                if not success:
                    executor.get_logger().error(f"双臂目标 {i + 1} 执行失败，本轮序列中止。")
                    break
            else:
                executor.get_logger().info(f"第 {loop_count} 轮双臂动作序列成功完成！")

    except KeyboardInterrupt:
        executor.get_logger().info('程序被手动中断 (Ctrl+C)')
    finally:
        executor.get_logger().info('正在关闭节点并清理资源...')
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()