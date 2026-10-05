import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


rclpy.init()
node = Node("recording_test_camera")
publisher = node.create_publisher(Image, sys.argv[1], 10)
announced = False


def publish():
    global announced
    message = Image()
    message.header.stamp = node.get_clock().now().to_msg()
    message.height = 2
    message.width = 2
    message.encoding = "rgb8"
    message.step = 6
    message.data = bytes([255, 0, 0] * 4)
    publisher.publish(message)
    if not announced and publisher.get_subscription_count():
        print("subscriber-connected", flush=True)
        announced = True


node.create_timer(0.05, publish)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
