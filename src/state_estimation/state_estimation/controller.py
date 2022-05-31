from cmath import pi
from math import atan2
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.goal = None
        self.position = None
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        self.recent_positions = list()
        
    def remove_old_deltas(self):
        while len(self.recent_positions) > 10:
            del self.recent_positions[0] # Delete oldest entry

    def timer_cb(self):
        if self.position is not None and self.goal is not None:
            # Calculate Delta Position (Postion -> Goal):
            # If delta_expected gets bigger when moving: Wrong direction 
            self.remove_old_deltas()
            facing_angle = 0
            if len(self.recent_positions)>=5:
                oldest = self.recent_positions[0]
                newest = self.recent_positions[-1]
                # Using trigonometry:
                facing_radian = pi + atan2(newest[1]-oldest[1], newest[0]-oldest[0]) #(x,y) Value Range: [-pi, +pi] --> [0, 2pi]
                facing_angle = (facing_radian / (2*pi))*360
                self.get_logger().info('Facing direction at the moment: {}'.format(facing_angle))

                    

        msg = Twist()
        #x = self.forward_distance - 0.3
        #x = x if x < 0.1 else 0.1
        #x = x if x >= 0 else 0.0
        # msg.linear.x = - 1
        # self.publisher.publish(msg)
    
    def goal_cb(self, msg):
        goal = msg.pose.position.x, msg.pose.position.y
        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
    
    def laser_cb(self, msg):
        self.forward_distance = msg.ranges[0]
        
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
