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
        self.servo_angle_matmsg = MycobotSetAngles()

        self.servo_angle_matmsg.joint_1 = 0.
        self.servo_angle_matmsg.joint_2 = 0.
        self.servo_angle_matmsg.joint_3 = 0.
        self.servo_angle_matmsg.joint_4 = 0.
        self.servo_angle_matmsg.joint_5 = 0.
        self.servo_angle_matmsg.joint_6 = 0.
        self.servo_angle_matmsg.speed = 10

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
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def angles_callback(self, msg):
        self.msg_servo_angle = msg

    def timer_callback(self):

        self.publisher_tgt_angles.publish(self.servo_angle_matmsg)
        self.publisher_heartbeat.publish(self.heartbeat_msg)
    

    def execute_callback(self, act_handle):
        self.get_logger().info('Executing action...')
        print(dir(act_handle))
        print(dir(act_handle.request.trajectory))
        print(' ')
        print('joint_names')
        print(act_handle.request.trajectory.joint_names)

        print(' ')
        print('joint_names')
        print(type(act_handle.request.trajectory.points))
        print(len(act_handle.request.trajectory.points))
        print(act_handle.request.trajectory.points[0])

        for point in act_handle.request.trajectory.points:
            print(" ")
            print(point)
            joints = np.array(point.positions) * 180 / np.pi

            self.servo_angle_matmsg.joint_1 = joints[0]  # degree
            self.servo_angle_matmsg.joint_2 = joints[1]
            self.servo_angle_matmsg.joint_3 = joints[2]
            self.servo_angle_matmsg.joint_4 = joints[3]
            self.servo_angle_matmsg.joint_5 = joints[4]
            self.servo_angle_matmsg.joint_6 = joints[5]
            self.servo_angle_matmsg.speed = 10

            print("self.servo_angle_matmsg")

            print(self.servo_angle_matmsg)

            # print(dir(point))
            self.publisher_tgt_angles.publish(self.servo_angle_matmsg)
            time.sleep(0.1)



        result = FollowJointTrajectory.Result()
        print(result)
        
        return result
    
    # def execute_callback1(self, goal_handle):
    #     self.get_logger().info('Executing goal...')
    #     self.get_logger().info(str(goal_handle.request.tgt_pose))

    #     reset_pose = goal_handle.request.tgt_pose
    #     self.reset_pose.set_trans(reset_pose[0], reset_pose[1], reset_pose[2])
    #     self.reset_pose.set_rot(reset_pose[3], reset_pose[4], reset_pose[5])

    #     self.get_logger().info('Executing goal...')

    #     feedback_msg = Reset.Feedback()

    #     for i in range(500):
    #         # self.get_logger().info('execute_callback_loop!')
    #         done, error = self.step_reset()
    #         feedback_msg.cur_pose = [error]
    #         goal_handle.publish_feedback(feedback_msg)
    #         time.sleep(0.1)
    #         if done:
    #             time.sleep(2.0)
    #             self.get_logger().info('reset done!')
    #             break

    #     goal_handle.succeed()
    #     result = Reset.Result()
        
    #     result.final_pose = [self.target_pose.translation[0], self.target_pose.translation[1], self.target_pose.translation[2],
    #                          self.target_pose.rotation[0], self.target_pose.rotation[1], self.target_pose.rotation[2]]
        
    #     return result



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