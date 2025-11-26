
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs  # This import is necessary for PoseStamped transformations

class MoveItPlanner(Node):
    def __init__(self):
        super().__init__('moveit_planner')

        # 1. Create a client for the GetPositionIK service
        self.ik_service_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting again...')

        # 2. Define the target pose
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'base_link'  # Or your desired frame
        self.target_pose.pose.position.x = 0.5
        self.target_pose.pose.position.y = 0.2
        self.target_pose.pose.position.z = 0.5
        self.target_pose.pose.orientation.w = 1.0

        # 3. Call the IK service
        self.call_ik_service()

    def call_ik_service(self):
        # Create the request
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator"  # Your MoveIt group name
        request.ik_request.pose_stamped = self.target_pose
        request.ik_request.timeout.sec = 5  # Timeout for the IK solver

        # Call the service
        future = self.ik_service_client.call_async(request)
        future.add_done_callback(self.ik_response_callback)

    def ik_response_callback(self, future):
        try:
            response = future.result()
            if response.error_code.val == response.error_code.SUCCESS:
                self.get_logger().info('IK solution found:')
                # You can now use the solution (response.solution) to move the robot
                # For example, by publishing to a joint trajectory controller
            else:
                self.get_logger().error(f'IK failed with error code: {response.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    moveit_planner = MoveItPlanner()
    rclpy.spin(moveit_planner)
    moveit_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
