import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher = self.create_publisher(String, 'controller_cmd', 10)
        self.get_logger().info('Controller node started')
        self.i = 0
        self.period = 1.0
        self.create_timer(self.period, self.timer_callback)
        self.state = "no movement"

    def timer_callback(self):
        if self.i % 20 < 5:
            self.state = "no movement"
        elif self.i % 20 < 10:
            self.state = "full movement"
        elif self.i % 20 < 15:
            self.state = "directional only"
        elif self.i % 20 < 20:
            self.state = "angular only"
        
        msg = String(data=self.state)
        self.publisher.publish(msg)
        self.get_logger().info(f'controller: {self.state}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    node = ControllerNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
