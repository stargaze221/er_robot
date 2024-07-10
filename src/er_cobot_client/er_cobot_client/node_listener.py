from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

import math

from er_cobot_interface_cmake.msg import MycobotAngles

class Listener(Node):

    def __init__(self):

        super().__init__('node_listener')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))


        self.create_subscription(MycobotAngles, '/platform_server/angles', self.callback, qos_profile)
        

        ### ROS2 Timer ###
        timer_period = 1/30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_controller']

    def callback(self, msg):
        joint1 =msg.joint_1* (math.pi / 180)
        joint2 =msg.joint_2* (math.pi / 180)
        joint3 =msg.joint_3* (math.pi / 180)
        joint4 =msg.joint_4* (math.pi / 180)
        joint5 =msg.joint_5* (math.pi / 180)
        joint6 =msg.joint_6* (math.pi / 180)
        self.joint_state.position = [joint1, joint2, joint3, joint4, joint5, joint6, 0.]
        print('callback', msg)

    def timer_callback(self):

        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_pub.publish(self.joint_state)
        
    


def main():
    rclpy.init()
    node_listener = Listener()
    rclpy.spin(node_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()