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
        self.current_pos = np.array([0.01,0.01,0.0]) # x^
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(0.2, self.timer_cb)
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
    

    def check_sol(self, x):
        # self.get_logger().info('Checking range Meassurements')
        for anchor in self.anchor_ranges:
            pos_anchor = np.array([anchor.anchor.x, anchor.anchor.y, anchor.anchor.z])
            estimate = np.linalg.norm(pos_anchor - x)
            # self.get_logger().info('Anchor at: {}; given distance: {}; Meassured Distance: {}; Delta: {}'.format(anchor.anchor, anchor.range, estimate, estimate-anchor.range))


    def calculate_position(self):
        if not len(self.anchor_ranges):
            return 0.0, 0.0, 0.0

        x_cfx = self.current_pos
        # self.get_logger().info("Anchor Ranges: {}".format(self.anchor_ranges))

        # YOUR CODE GOES HERE:

        # Array: [(msg.range, msg.Point)]

        # Residuals: 
        residual = np.array([])

        # self.get_logger().info("Calculating residual")
        anchor_counter = 0
        for anchor in self.anchor_ranges:
            #self.get_logger().info("anchor [{}] x: {}, y: {}, z: {}, range: {}".format(anchor_counter, anchor.anchor.x, anchor.anchor.y, anchor.anchor.z, anchor.range))
    
            pos_anchor = np.array([anchor.anchor.x, anchor.anchor.y, anchor.anchor.z])
            anchor_residual = anchor.range - np.linalg.norm(pos_anchor - x_cfx)
            #self.get_logger().info("Residual for anchor [{}]: {}".format(anchor_counter, anchor_residual))
            residual = np.append(residual, anchor_residual)
            anchor_counter += 1

        res_gradient = np.array([])

        # self.get_logger().info("Calculating gradient for residual")
        # Gradient des Residuals:
        anchor_counter = 0
        for anchor in self.anchor_ranges:
            #self.get_logger().info("anchor [{}] x: {}, y: {}, z: {}, range: {}".format(anchor_counter, anchor.anchor.x, anchor.anchor.y, anchor.anchor.z, anchor.range))
            pos_anchor = np.array([anchor.anchor.x, anchor.anchor.y, anchor.anchor.z])
            denom = np.linalg.norm(x_cfx - pos_anchor)
            #self.get_logger().info("denom: {}".format(denom))
            gradient_row = np.array([])
            if denom == 0: 
                self.get_logger().error("Zero division. Stuff is scuffed here")
            else:
                for a,b in zip(x_cfx, pos_anchor):
                    nom = -(a-b)
                    #self.get_logger().info("Nominator (a-b): {} ({} - {})".format(nom, a, b))
                    gradient_row = np.append(gradient_row, nom/denom)
                    #self.get_logger().info("Division result: {}".format(nom/denom))
            res_gradient = np.append(res_gradient, gradient_row, axis=0)
            #self.get_logger().info("Residum gradient: {}".format(res_gradient))
            anchor_counter += 1
        res_gradient = res_gradient.reshape(len(self.anchor_ranges), 3)
        #self.get_logger().info("Residum gradient after reshape: {}".format(res_gradient))
        # Pseudo-inverse of residual gradient
        res_gradient_pseudo_inv = np.linalg.pinv(res_gradient)

        #self.get_logger().info("Pseduo-inverse of residum gradient: {}".format(res_gradient_pseudo_inv))

        sol = x_cfx - (res_gradient_pseudo_inv @ residual)

        #self.get_logger().info("matrix multiplication: {}".format(res_gradient_pseudo_inv @ residual))

        #self.get_logger().info("result: {}".format(sol))

        #self.get_logger().error("sol: {}; Using approx. Position: {}, diff: {}".format(sol, x_cfx, sol-x_cfx))

        self.current_pos = sol # Big Scuffed
        self.check_sol(self.current_pos)
        return self.current_pos



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
