#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped


class TofNode(Node):
    def __init__(self):
        super().__init__('tof')
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.movingback = False
        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.check_range, 10)
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)
        self.counter = 0
        self.timer = self.create_timer(1, self.check_range)

    def check_range(self, msg):
        distance = msg.range
        if distance >= 0.2 and not self.movingback:
            self.move_forward()
        elif self.movingback and self.counter <= 5:
            self.move_backward()
        else:
            self.stop()
            self.movingback = True


    def move_forward(self):
        self.run_wheels('forward_callback', 0.5, 0.5)

    def move_backward(self):

        self.run_wheels('backward_callback', -0.5, -0.5)
        self.counter += 1

    def stop(self):
        self.run_wheels('stop_callback', 0.0, 0.0)

    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheels_pub.publish(wheel_msg)


def main():
    rclpy.init()
    tof = TofNode()
    rclpy.spin(tof)
    rclpy.shutdown()


if __name__ == '__main__':
    main()