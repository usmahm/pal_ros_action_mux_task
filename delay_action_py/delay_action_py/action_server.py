from action_interfaces.srv import String

import rclpy
from rclpy.node import Node

class ActionServer(Node):
    def __init__(self):
        super().__init__("action_server")
        self.service_ = self.create_service(String, 'action', self.action_delay_cb)
        self.timer = None

        self.get_logger().info("Action Service Ready")

    def action_delay_cb(self, request, response):
        if self.timer is not None:
            self.get_logger().info("Cancelling previous goal")
            self.timer.cancel()
            self.timer = None
        
        self.get_logger().info("Scheduling new future goal. Goal: '%s'" % request.data)
            
        self.timer = self.create_timer(5, lambda : self.timer_cb(request))

        response.output = "Requested goal scheduled. Goal: '%s'" % request.data
        return response

    def timer_cb(self, request):
        self.action_cb(request)

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

    def action_cb(self, request):
        self.get_logger().info("Goal Activated: '%s'" % request.data)

def main(args=None):
    rclpy.init(args=args)

    action_server = ActionServer()

    rclpy.spin(action_server)
    
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
