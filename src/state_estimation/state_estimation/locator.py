from turtle import pos
import rclpy
import numpy as np
from rclpy.node import Node

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped


class LocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')
        
    def range_cb(self, msg):
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-10:]
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)



    def get_np_arr_from_point(x,y,z):
        return np.array([x, y, z])
    
    def calculate_position(self):
        if not len(self.anchor_ranges):
            return 0.0, 0.0, 0.0
        self.get_logger().info("Anchor Ranges: {}".format(self.anchor_ranges))

        # YOUR CODE GOES HERE:

        # Array: [(msg.range, msg.Point)]


        aprox = np.array([1.33213,1.432,1.32167]) # x^


        # Residuals: 
        residual = np.array([])

        for anchor in self.anchor_ranges:
            # self.get_logger().error("Anchor.anchor: ".format(print(dir(anchor.anchor))))
            # self.get_logger().info("anchor.anchor.x/y/z: x: {}, y: {}: z: {}".format(anchor.anchor.x, anchor.anchor.y, anchor.anchor.z))
            pos_anchor = np.array([anchor.anchor.x, anchor.anchor.y, anchor.anchor.z])
            temp = anchor.range - np.linalg.norm(pos_anchor - aprox)
            residual = np.append(residual, temp)


        res_gradient = np.array([])

        # Gradient des Residuals:
        for anchor in self.anchor_ranges:
            temp = np.array([])
            pos_anchor = np.array([anchor.anchor.x, anchor.anchor.y, anchor.anchor.z])
            denom = np.linalg.norm(aprox - pos_anchor)
            if(denom == 0): self.get_logger().error("Zero division. Stuff is scuffed here")
            for a,b in zip(aprox, pos_anchor):
                temp = np.append(temp, (a-b)/denom)
            self.get_logger().info("temp: {}".format(temp))
            res_gradient = np.append(res_gradient, temp, axis=0)

        res_gradient = res_gradient.reshape(len(self.anchor_ranges), 3)

        self.get_logger().error("res_gradiant: {}".format(res_gradient))
        
        # Pseudo-inverse of residual gradient
        res_gradient_pseudo_inv = np.linalg.pinv(res_gradient)
        self.get_logger().error("res_gradiant_pseudo_inv: {}".format(res_gradient_pseudo_inv))

        sol = aprox - (res_gradient_pseudo_inv @ residual)

        self.get_logger().error("sol: {}".format(sol))

        return 1.0, 0.0, 0.0



def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
