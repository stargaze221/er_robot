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

        ### Subscribers ###
        self.create_subscription(MycobotAngles, '/platform_server/angles', self.angles_callback, 10)
        self.msg_servo_angle = None

        ### ROS2 Timer ###
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def angles_callback(self, msg):
        self.msg_servo_angle = msg

    def timer_callback(self):

        # Set Servo Angles
        if self.msg_servo_angle is not None:
            # angles = pack_np_matrix_from_multiarray_msg(self.msg_servo_angle).squeeze()     
            print('angles:', self.msg_servo_angle)

        # Publish Servo Angles
        self.servo_angle_matmsg.joint_1 = 60.
        self.servo_angle_matmsg.joint_2 = 0.
        self.servo_angle_matmsg.joint_3 = 0.
        self.servo_angle_matmsg.joint_4 = 0.
        self.servo_angle_matmsg.joint_5 = 0.
        self.servo_angle_matmsg.joint_6 = 0.
        self.servo_angle_matmsg.speed = 20

        self.publisher_tgt_angles.publish(self.servo_angle_matmsg)

        self.publisher_heartbeat.publish(self.heartbeat_msg)



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