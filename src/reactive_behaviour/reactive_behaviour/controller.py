from cmath import pi
from math import nan
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
        super().__init__("velocity_controller")
        self.declare_parameter("rand_turn_prob")
        self.random_spin_probability = self.get_parameter("rand_turn_prob").get_parameter_value().double_value
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_message = None
        self.front_view = None
        #mself.random_spin_probability = 0.02 # ToDo: Pass random_spin_prob via Command line argument
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("Controller node started Successfully with random_spin_probability: {}".format(self.random_spin_probability))
        
        
    # Returns new angle for robot, with its collision condition being false
    def get_random_angle(self):
        rand = randint(1, 359)
        while self.current_message[rand] < COLLISION_DISTANCE:
            rand = randint(1, 359)
        return (2 * pi) / (360/rand)

    def timer_cb(self):
        msg = Twist()
        v = 0.0
        if self.front_view is not None:
            # If About to crash -> Change Angle to another random angle (If this angle's collision condition is false)
            if min(self.front_view) < COLLISION_DISTANCE:
                msg.angular.z = self.get_random_angle()
            elif self.random_spin_probability != 0.0 and random() < self.random_spin_probability:
                self.get_logger().info("I've made a random turn")
                msg.angular.z = self.get_random_angle()
            else:
                v = VELOCITY

        msg.linear.x = v

        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        # Save range data in class attribute & save front angles in specified array
        self.current_message = msg.ranges
        self.front_view = msg.ranges[360-ANGLE_HALF:360]
        self.front_view += msg.ranges[0:ANGLE_HALF]
        for i in range(0, len(self.front_view)):
            if self.front_view[i] == 0: self.front_view[i] = nan




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
