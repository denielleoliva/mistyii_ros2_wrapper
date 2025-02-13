import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import requests

class MistyROS2Wrapper(Node):
    def __init__(self):
        super().__init__('misty_ros2_wrapper')

        # Define Misty's API base URL (Modify with Misty's IP address)
        self.misty_ip = '192.168.1.100'  # Change accordingly
        self.base_url = f'http://{self.misty_ip}/api'

        # publishers
        self.battery_pub = self.create_publisher(Float32, 'misty/battery', 10)
        self.speech_pub = self.create_publisher(String, 'misty/speech_recognition', 10)

        # subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, 'misty/speak', self.speak_callback, 10)
        self.create_subscription(String, 'misty/led', self.led_callback, 10)

        # timers
        self.create_timer(10.0, self.publish_battery_status)

    def cmd_vel_callback(self, msg):
        # twist msg for misty api
        data = {"LinearVelocity": msg.linear.x, "AngularVelocity": msg.angular.z}
        requests.post(f'{self.base_url}/drive', json=data)
        self.get_logger().info(f'Moving Misty: {data}')

    def speak_callback(self, msg):
        data = {"Text": msg.data}
        requests.post(f'{self.base_url}/speak', json=data)
        self.get_logger().info(f'Misty speaking: {msg.data}')

    def led_callback(self, msg):
        data = {"red": 255, "green": 0, "blue": 0}  
        requests.post(f'{self.base_url}/led', json=data)
        self.get_logger().info(f'Misty LED set to red')

    def publish_battery_status(self):
        response = requests.get(f'{self.base_url}/battery')
        if response.status_code == 200:
            battery_level = response.json().get("chargePercent", 0.0)
            msg = Float32()
            msg.data = float(battery_level)
            self.battery_pub.publish(msg)
            self.get_logger().info(f'Battery: {battery_level}%')
        else:
            self.get_logger().warn('Failed to retrieve battery status.')


def main(args=None):
    rclpy.init(args=args)
    node = MistyROS2Wrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
