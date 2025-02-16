import rclpy
from rclpy.node import Node
from annex_msgs.msg import Vcu2ai, Ai2vcu


class LidarNode(Node):
    def __init__(self):
        super().__init__('path_planning')


# main method
def main(args=None):
    rclpy.init(args=args)

    lidar = LidarNode()

    rclpy.spin(lidar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


