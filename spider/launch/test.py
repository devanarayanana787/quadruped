import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from enum import Enum

# Define the states for our simple walking gait
class GaitState(Enum):
    LIFTING_A = 0
    SWINGING_A = 1
    LIFTING_B = 2
    SWINGING_B = 3

class SpiderGaitCommander(Node):
    def __init__(self):
        super().__init__('spider_gait_commander')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)
        
        # Gait parameters
        gait_period = 0.5  # seconds for each phase of the gait
        self.timer = self.create_timer(gait_period, self.gait_callback)
        self.current_state = GaitState.LIFTING_A # Start with the first state

        # Define the joint names in the correct order
        self.joint_names = [
            'f1_joint', 'f11_joint', 'f111_joint',  # Front-Left (FL)
            'f2_joint', 'f22_joint', 'f222_joint',  # Front-Right (FR)
            'b1_joint', 'b11_joint', 'b111_joint',  # Back-Left (BL)
            'b2_joint', 'b22_joint', 'b222_joint'   # Back-Right (BR)
        ]

        # ❗ TUNE THESE JOINT ANGLES FOR YOUR ROBOT ❗
        # Angles are [coxa, femur, tibia]
        self.LEG_POSES = {
            "down_forward": [0.0, 0.7, -1.5],
            "up":           [0.0, 0.3, -1.0], # Lifted position
            "down_backward": [0.0, -0.7, 1.5],
        }
        
        self.get_logger().info('Spider Gait Commander has been started.')

    def create_trajectory_point(self, positions, time_from_start_sec):
        """Helper function to create a JointTrajectoryPoint."""
        point = JointTrajectoryPoint()
        point.positions = positions
        # THIS LINE IS NOW FIXED
        point.time_from_start = Duration(sec=int(time_from_start_sec), nanosec=int((time_from_start_sec % 1) * 1e9))
        return point

    def gait_callback(self):
        """The main state machine callback for the walking gait."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        fl_pose = []
        fr_pose = []
        bl_pose = []
        br_pose = []

        if self.current_state == GaitState.LIFTING_A:
            self.get_logger().info("State: LIFTING GROUP A (FL, BR)")
            fl_pose = self.LEG_POSES["up"]
            br_pose = self.LEG_POSES["up"]
            fr_pose = self.LEG_POSES["down_forward"]
            bl_pose = self.LEG_POSES["down_forward"]
            self.current_state = GaitState.SWINGING_A
        
        elif self.current_state == GaitState.SWINGING_A:
            self.get_logger().info("State: SWINGING GROUP A (FL, BR)")
            fl_pose = self.LEG_POSES["down_forward"]
            br_pose = self.LEG_POSES["down_forward"]
            fr_pose = self.LEG_POSES["down_backward"]
            bl_pose = self.LEG_POSES["down_backward"]
            self.current_state = GaitState.LIFTING_B
            
        elif self.current_state == GaitState.LIFTING_B:
            self.get_logger().info("State: LIFTING GROUP B (FR, BL)")
            fr_pose = self.LEG_POSES["up"]
            bl_pose = self.LEG_POSES["up"]
            fl_pose = self.LEG_POSES["down_forward"]
            br_pose = self.LEG_POSES["down_forward"]
            self.current_state = GaitState.SWINGING_B
            
        elif self.current_state == GaitState.SWINGING_B:
            self.get_logger().info("State: SWINGING GROUP B (FR, BL)")
            fr_pose = self.LEG_POSES["down_forward"]
            bl_pose = self.LEG_POSES["down_forward"]
            fl_pose = self.LEG_POSES["down_backward"]
            br_pose = self.LEG_POSES["down_backward"]
            self.current_state = GaitState.LIFTING_A

        target_positions = fl_pose + fr_pose + bl_pose + br_pose
        
        point = self.create_trajectory_point(target_positions, 0.4) # Take 0.4s for this movement
        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    gait_commander = SpiderGaitCommander()
    rclpy.spin(gait_commander)
    gait_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()