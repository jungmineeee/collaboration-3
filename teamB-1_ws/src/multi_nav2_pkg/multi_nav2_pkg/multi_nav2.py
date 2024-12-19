import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class TurtleBotNav2Controller(Node):
    def __init__(self):
        super().__init__('multi_nav2_node')

        # Action clients for each turtlebot
        self.turtlebot1_nav_client = ActionClient(
            self, NavigateToPose, '/turtle1/navigate_to_pose'
        )
        self.turtlebot2_nav_client = ActionClient(
            self, NavigateToPose, '/turtle2/navigate_to_pose'
        )

        # Example goal coordinates for each turtlebot
        self.turtlebot1_goal = PoseStamped()
        self.turtlebot1_goal.header.frame_id = 'map'
        self.turtlebot1_goal.pose.position.x = 1.174290657043457
        self.turtlebot1_goal.pose.position.y = -8.401721000671387
        self.turtlebot1_goal.pose.orientation.w = 1.0

        self.turtlebot2_goal = PoseStamped()
        self.turtlebot2_goal.header.frame_id = 'map'
        self.turtlebot2_goal.pose.position.x = 8.44029426574707
        self.turtlebot2_goal.pose.position.y = -12.985867500305176
        self.turtlebot2_goal.pose.orientation.w = 1.0

        # Send goals to both turtlebots
        self.send_goals()

    def send_goals(self):
        self.send_goal(self.turtlebot1_nav_client, self.turtlebot1_goal, 'turtlebot1')
        self.send_goal(self.turtlebot2_nav_client, self.turtlebot2_goal, 'turtlebot2')

    def send_goal(self, client, pose, turtlebot_name):
        # Wait for the action server
        client.wait_for_server()
        self.get_logger().info(f'{turtlebot_name} action server ready, sending goal.')

        # Create NavigateToPose goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # Send goal asynchronously
        send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, turtlebot_name)
        )

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f"Feedback: Current robot position (partial data): {feedback_msg.feedback.current_pose.pose.position}"
        )

    def goal_response_callback(self, future, turtlebot_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'{turtlebot_name} goal rejected!')
            return

        self.get_logger().info(f'{turtlebot_name} goal accepted, monitoring progress...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.result_callback(future, turtlebot_name)
        )

    def result_callback(self, future, turtlebot_name):
        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(f'{turtlebot_name} reached the goal!')
        else:
            self.get_logger().info(f'{turtlebot_name} failed to reach the goal.')


def main(args=None):
    rclpy.init(args=args)
    turtlebot_nav2_controller = TurtleBotNav2Controller()

    try:
        rclpy.spin(turtlebot_nav2_controller)
    except KeyboardInterrupt:
        pass
    finally:
        turtlebot_nav2_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

