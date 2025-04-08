# from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

class GenericSubscriber(Node):
    def __init__(self, topic_name: str):
        super().__init__("generic_subscriber")

        self.topic_name_ = topic_name

        self.timer_ = self.create_timer(5, self.attempt_sub_to_topic)

        self.get_logger().info(f"Generic Subscriber Initialized on topic: '{topic_name}'")
    
    def attempt_sub_to_topic(self):
        topic_publishers_info = self.get_publishers_info_by_topic(self.topic_name_)

        if len(topic_publishers_info) != 0:
            self.timer_.cancel()
            self.timer_ = None

            # Only subscribe to the first one since ROS2 can only subscribe to a
            # single topic-type pair, can't sub to different types on same topic
            first_topic_info = topic_publishers_info[0]
            msg_type_str = first_topic_info.topic_type

            MsgType = get_message(msg_type_str)
            if MsgType is None:
                self.get_logger().error(f"Failed to import message type for '{msg_type_str}'")
                return
            
            self.get_logger().info(f"Subscribing to topic '{self.topic_name_}' with type: {msg_type_str}")
            self.subscription_ = self.create_subscription(MsgType, self.topic_name_, lambda msg : self.sub_cb(msg, msg_type_str), 10)
    
    def sub_cb(self, msg, msg_type):
      self.get_logger().info(f"Received a message from topic '{self.topic_name_}' with type: {msg_type}")

def main(args=None):
    rclpy.init(args=args)

    generic_sub = GenericSubscriber("/buffer")

    rclpy.spin(generic_sub)
    
    generic_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
