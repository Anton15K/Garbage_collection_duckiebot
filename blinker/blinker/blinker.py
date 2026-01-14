import os
import rclpy
from rclpy.node import Node
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA


class RedLED(Node):
    def __init__(self):
        super().__init__('red_led')

        self.vehicle_name = os.getenv('VEHICLE_NAME')
        if not self.vehicle_name:
            raise RuntimeError("VEHICLE_NAME environment variable not set")

        self.publisher = self.create_publisher(
            LEDPattern,
            f'/{self.vehicle_name}/led_pattern',
            1
        )

        # Publish once per second (keeps LEDs solid)
        self.timer = self.create_timer(1.0, self.publish_pattern)

    def publish_pattern(self):
        msg = LEDPattern()
        msg.pattern_name = "solid_red"
        msg.frequency = 1.0

        red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        # Duckiebot has 5 LEDs
        msg.rgb_vals = [red] * 5

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = RedLED()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
