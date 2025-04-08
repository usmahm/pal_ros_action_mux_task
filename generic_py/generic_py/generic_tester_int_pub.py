from std_msgs.msg import Int64

import rclpy
from rclpy.node import Node

class GenericTesterIntPub(Node):

    def __init__(self, topic_name):
        super().__init__('generic_tester_int_pub')

        self.publisher_ = self.create_publisher(Int64, topic_name, 10)
        
        self.timer = self.create_timer(5.0, self.publish_messages)

    def publish_messages(self):
        msg = Int64()
        msg.data = 30
        
        self.get_logger().info(f"Publishing integer message type of value: '{msg.data}'")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    generic_tester_pub = GenericTesterIntPub("/buffer")

    rclpy.spin(generic_tester_pub)

    generic_tester_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()