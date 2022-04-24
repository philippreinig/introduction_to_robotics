from cmath import pi
from random import randint, random
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

ANGLE_HALF = 25
VELOCITY = 0.1
COLLISION_DISTANCE = 0.3

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_message = None
        self.front_view = None
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        
    def timer_cb(self):
        msg = Twist()
        v = VELOCITY
        if self.front_view is not None and min(self.front_view) < COLLISION_DISTANCE:
            v = 0.0
            rand = randint(1, 359)
            while self.current_message[rand] < COLLISION_DISTANCE:
                rand = randint(1, 359)
            msg.angular.z = (2 * pi) / (360/rand)

        msg.linear.x = v

        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        # Save range data in class attribute & save front angles in specified array
        self.current_message = msg.ranges
        self.front_view = msg.ranges[360-ANGLE_HALF:360]
        self.front_view += msg.ranges[0:ANGLE_HALF]
        # self.get_logger().error('{}'.format(len(msg.ranges)))




def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
