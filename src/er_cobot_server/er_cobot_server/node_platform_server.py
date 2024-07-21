import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from pymycobot.mycobot import MyCobot
from er_cobot_interface_cmake.msg import MycobotSetAngles
from er_cobot_interface_cmake.msg import MycobotAngles  

from control_msgs.action import FollowJointTrajectory
import numpy as np

import time

class NodePlatform(Node):

    def __init__(self):
        super().__init__('node_platform_server')
        port = '/dev/ttyAMA0'
        baud = 1000000
        self.mc = MyCobot(port, baud)
        ### Publishers ###
        self.pub_angles = self.create_publisher(MycobotAngles, 'platform_server/angles', 10)
        self.msg_angles = MycobotAngles()
        ### Subscribers ###
        self.create_subscription(MycobotSetAngles, 'platform_client/tgt_angles', self.callback_tgt, 10)
        self.msg_tgt_angles = None
        self.timer = self.create_timer(0.1, self.timer_callback)

        ### Initial poistion ###
        initial_position = [0., 0., 0., 0., 0., 0.]
        sp =  10
        self.mc.send_angles(initial_position, sp)

        ### Action Server ###
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing action...')

        feedback_msg = FollowJointTrajectory.Feedback()

        for desired_point in goal_handle.request.trajectory.points:

            # desired postions
            np_desired_positions = np.array(desired_point.positions)
            feedback_msg.desired.positions = np_desired_positions.tolist()

            # send angle command to MyCobot (mc)
            tgt_angles = np_desired_positions * (180 / np.pi)
            sp = 10
            self.mc.send_angles(tgt_angles.tolist(), sp)

            # get actual angle
            np_actual_positions = np.array(self.mc.get_angles()) * (np.pi / 180)
            feedback_msg.actual.positions = np_actual_positions.tolist()

            # error positions
            np_error_positions = np_desired_positions - np_actual_positions
            feedback_msg.error.positions = np_desired_positions.tolist()
            error = np.linalg.norm(np_error_positions)

            while error > 0.1:
                print('error', error)
                np_actual_positions = np.array(self.mc.get_angles()) * (np.pi / 180)
                np_error_positions = np_desired_positions - np_actual_positions
                error = np.linalg.norm(np_error_positions)
                self.mc.send_angles(tgt_angles.tolist(), sp)
                time.sleep(.1)


        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = int(0)
        
        return result

    def callback_tgt(self, msg):
        self.msg_tgt_angles = msg

    def timer_callback(self):

        try:
            angles = self.mc.get_angles()

            self.msg_angles.joint_1 = angles[0]
            self.msg_angles.joint_2 = angles[1]
            self.msg_angles.joint_3 = angles[2]
            self.msg_angles.joint_4 = angles[3]
            self.msg_angles.joint_5 = angles[4]
            self.msg_angles.joint_6 = angles[5]

            self.pub_angles.publish(self.msg_angles)

        except:
            print("error in getting servo angles")


        if self.msg_tgt_angles is not None:
            ang = self.msg_tgt_angles
            angles = [ang.joint_1, ang.joint_2, ang.joint_3, ang.joint_4, ang.joint_5, ang.joint_6]
            sp = ang.speed
            self.mc.send_angles(angles, sp)

def main():
    rclpy.init()
    node_platform = NodePlatform()
    rclpy.spin(node_platform)

    # Destroy the node
    node_platform.destroy_node()
    node_platform.mc.set_color(255,255,255)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
