from action_interfaces.srv import String as AString
from std_msgs.msg import String

import rclpy
from rclpy.node import Node

class ActionClient(Node):
    def __init__(self):
        super().__init__("action_client")
        self.client_ = self.create_client(AString, "action")
        
        while not self.client_.wait_for_service(1.0):
            if not rclpy.ok():
                self.get_logger().info("Interrupted while waiting for the service. Exiting.")
                return
            self.get_logger().info("action service not available, waiting again...")
        
        self.subscriber_ = self.create_subscription(String, "new_action", self.action_sub_cb, 10)

        self.get_logger().info("Action Client Ready. Listening on 'new_action' topic")

    
    def action_sub_cb(self, msg: String):
        self.get_logger().info("New action received from subscriber. Action: '%s'" % msg.data)
        
        self.send_request(msg.data)

    def send_request(self, input: str):
        req = AString.Request()
        req.data = input

        future = self.client_.call_async(req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        response = future.result()
        self.get_logger().info("Output: %s" % response.output)
    
def main(args=None):
    rclpy.init(args=args)

    action_client = ActionClient()

    rclpy.spin(action_client)
    
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        
