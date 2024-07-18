import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray        # See https://gist.github.com/jarvisschultz/7a886ed2714fac9f5226
from std_msgs.msg import MultiArrayDimension      # See http://docs.ros.org/api/std_msgs/html/msg/MultiArrayLayout.html

from sensor_msgs.msg import Imu

from er_cobot_interface_cmake.msg import MycobotAngles
from er_cobot_interface_cmake.msg import MycobotSetAngles

from .submodules.msgpacking import init_matrix_array_ros_msg, pack_multiarray_ros_msg, pack_np_matrix_from_multiarray_msg

import numpy as np


from control_msgs.action import FollowJointTrajectory


from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from rclpy.action import ActionServer

import time


def pack_np_array_from_MycobotAngles(cobotangles):
    array = [ cobotangles.joint_1,
              cobotangles.joint_2,
              cobotangles.joint_3,
              cobotangles.joint_4,
              cobotangles.joint_5,
              cobotangles.joint_6]
    return np.array(array)

def update_angles_msg_from_np_angles(msg, joint_angles):
    msg.joint_1 = joint_angles[0]
    msg.joint_2 = joint_angles[1]
    msg.joint_3 = joint_angles[2]
    msg.joint_4 = joint_angles[3]
    msg.joint_5 = joint_angles[4]
    msg.joint_6 = joint_angles[5]
    return 0

def update_set_angles_msg_from_angles_speed(msg, joint_angles, speed):
    update_angles_msg_from_np_angles(msg, joint_angles)
    msg.speed = speed
    return 0



#################
### ROS2 Node ###
#################
class NodePlatform(Node):

    def __init__(self):
        super().__init__('node_platform_client')

        qos_profile = QoSProfile(depth=10)

        ### Publishers ###
        self.publisher_tgt_angles = self.create_publisher(MycobotSetAngles, '/platform_client/tgt_angles', qos_profile)
        self.publisher_heartbeat = self.create_publisher(String, '/platform/heartbeat', qos_profile)
        self.heartbeat_msg = String()
        self.heartbeat_msg.data = "OK"
        self.set_angle_msg = MycobotSetAngles()

        # Set initial position
        initial_pos = np.array([0.,0.,0.,0.,0.,0.])
        update_set_angles_msg_from_angles_speed(self.set_angle_msg, initial_pos, 10)

        ### Subscribers ###
        self.create_subscription(MycobotAngles, '/platform_server/angles', self.angles_callback, 10)
        self.msg_servo_angle = None

        ### Action Server ###
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_callback)

        ### ROS2 Timer ###
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def angles_callback(self, msg):
        print('subsribed!')
        self.msg_servo_angle = msg

    def timer_callback(self):
        self.publisher_tgt_angles.publish(self.set_angle_msg)
        self.publisher_heartbeat.publish(self.heartbeat_msg)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing action...')

        feedback_msg = FollowJointTrajectory.Feedback()
        
        for desired_point in goal_handle.request.trajectory.points:

            # desired postions
            np_desired_positions = np.array(desired_point.positions)
            feedback_msg.desired.positions = np_desired_positions.tolist()
            

            while True:

                # actual positions
                np_actual_positions = pack_np_array_from_MycobotAngles(self.msg_servo_angle) * (np.pi / 180)
                feedback_msg.actual.positions = np_actual_positions.tolist()
                print('actual:', np_actual_positions)
                print('desired:', np_desired_positions)
                
                # error positions
                np_error_positions = np_desired_positions -np_actual_positions
                feedback_msg.error.positions = np_desired_positions.tolist()
                error = np.linalg.norm(np_error_positions)

                update_set_angles_msg_from_angles_speed(self.set_angle_msg, np_desired_positions * (180 / np.pi), 20)
                self.publisher_tgt_angles.publish(self.set_angle_msg)
                time.sleep(.1)

                print('error:',error)
                
                if error < 0.2:
                    break

            update_set_angles_msg_from_angles_speed(self.set_angle_msg, np_desired_positions * (180 / np.pi), 20)
            self.publisher_tgt_angles.publish(self.set_angle_msg)            

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = int(0)
        
        return result


def main():
    rclpy.init()
    node_platform = NodePlatform()
    rclpy.spin(node_platform)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_platform.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()