import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class WatchdogNode(Node):

    

    def __init__(self):
        super().__init__('watchdog')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'input_cmd', self.cmd_callback, 10)
        self.create_subscription(String, 'controller_cmd', self.controller_callback, 10)
        self.get_logger().info('Watchdog node started')
        self.controller_state = "no movement"

    def cmd_callback(self, msg):
        #self.get_logger().info(f'cmd_callback~msg: {msg}')
        # this makes the turle go backwards
        # (just so you know its working)
        if (self.controller_state == "full movement"):
            pass
        elif (self.controller_state == "angular only"):
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
        elif (self.controller_state == "directional only"):
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
        elif (self.controller_state == "no movement"):
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
        else:
            raise ValueError(f"Not handling controller_state: {self.controller_state}")

        # self.get_logger().info(f'msg.linear.x: {msg.linear.x}')
        #self.get_logger().info(f'cmd_callback: controller state: {self.controller_state}, message published: {msg}')
        self.publisher.publish(msg)
        
    def controller_callback(self, msg):
        # self.get_logger().info('Controller callback called:', msg)
        self.controller_state = msg.data
        self.get_logger().warn(f'Watchdog: Reveived state update from controller: {msg.data}')



def main(args=None):
    rclpy.init(args=args)

    node = WatchdogNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
