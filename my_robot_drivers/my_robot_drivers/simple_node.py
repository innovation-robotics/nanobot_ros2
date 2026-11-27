#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleRobotNode(Node):
    def __init__(self):
        super().__init__('simple_robot_node')
        # Add subscribers, publishers, services, or action servers here
        self.get_logger().info('SimpleRobotNode started.')
        self.cmd_vel_sub = self.create_subscription(Twist,'/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        pass
    
def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()