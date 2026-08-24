"""Publish the odometry transform for the Gazebo-simulated Traymover."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class SimOdomTf(Node):
    def __init__(self) -> None:
        super().__init__('sim_odom_tf')
        self._broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

    def odom_callback(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header = msg.header
        # Keep this node's TF contract independent of Gazebo's frame labels.
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimOdomTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
