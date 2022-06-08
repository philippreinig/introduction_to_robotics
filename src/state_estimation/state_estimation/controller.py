from cmath import nan, pi
from math import atan2
from random import randint, random
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan

COLLISION_DISTANCE = 0.3
VELOCITY = 0.1
ANGLE_HALF = 25
DEBUG = False

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.goal = None
        self.position = None
        self.front_view = None
        self.target_angle = None
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        self.recent_positions = list()
        
    def remove_old_positions(self):
        while len(self.recent_positions) > 4:
            del self.recent_positions[0] # Delete oldest entry

    def timer_cb(self):
        if not DEBUG:
            msg = Twist()
            if self.position is not None and self.goal is not None:
                # Calculate Delta Position (Postion -> Goal):
                # If delta_expected gets bigger when moving: Wrong direction
                msg.linear.x = 0.1
                self.remove_old_positions()
                if len(self.recent_positions)>=3:
                    oldest = self.recent_positions[0]
                    newest = self.recent_positions[-1]
                    # Using trigonometry to get current facing (oldest and newest pos):
                    facing_radian = pi + atan2(newest[1]-oldest[1], newest[0]-oldest[0]) #(y,x) Value Range: [-pi, +pi] --> [0, 2pi]
                    # self.get_logger().info('Facing direction at the moment: {}'.format(facing_angle))
                    # Using trigonometry to get target angle (newest pos and goal):          
                    target_radian = pi + atan2(self.goal[1]-newest[1], self.goal[0]-newest[0]) #(y,x) Value Range: [-pi, +pi] --> [0, 2pi]
                    self.get_logger().info('AngleToGoal: {}; Calc.Angle {}'.format(target_radian, facing_radian))
                    turn_angle = target_radian - facing_radian
                    # self.target_angle = target_radian
                    msg.angular.z = turn_angle
            #x = self.forward_distance - 0.3
            #x = x if x < 0.1 else 0.1
            #x = x if x >= 0 else 0.0
            # msg.linear.x = - 1
            msg = self.collision_detection_behavior(msg)
            self.publisher.publish(msg)


    def get_random_angle(self):
        rand = randint(1, 359)
        while self.current_message[rand] < COLLISION_DISTANCE:
            rand = randint(1, 359)
        return (2 * pi) / (360/rand)


    def collision_detection_behavior(self, msg):   
        if self.front_view is not None:
            # If About to crash -> Change Angle to another random angle (If this angle's collision condition is false)
            if min(self.front_view) < COLLISION_DISTANCE:
                msg.angular.z = self.get_random_angle()
                self.recent_positions = list()
        return msg


    def goal_cb(self, msg):
        goal = msg.pose.position.x, msg.pose.position.y
        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
    
    def laser_cb(self, msg):
        # Save range data in class attribute & save front angles in specified array
        self.current_message = msg.ranges
        self.front_view = msg.ranges[360-ANGLE_HALF:360]
        self.front_view += msg.ranges[0:ANGLE_HALF]
        for i in range(0, len(self.front_view)):
            if self.front_view[i] == 0: self.front_view[i] = nan
        
    def position_cb(self, msg):
        self.position = msg.point.x, msg.point.y
        self.recent_positions.append((msg.point.x, msg.point.y))


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
