import rclpy
from rclpy.node import Node
from annex_msgs.msg import Vcu2ai, Ai2vcu
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_prefix
import os

PKG_NAME = "motion_package"
LOG = "data_logs"
FILE = "odometry.csv"


class GazeboOdometryNode(Node):
    def __init__(self):
        super().__init__('Odometry_node')
        # initiate vars
        self.logger = self.get_logger()
        # initiate log file
        self.log_file = os.path.join(get_package_prefix(PKG_NAME),LOG, FILE)
        self.mode = "a+" if os.path.isfile(self.log_file) else "w+"

    # subscription and publishing
    def _sub_pub(self):
        self.create_subscription(Odometry, "model/adsmt/odometry", self.odometry_callback, 10)

    # IMU callback: called everytime the IMU data is published
    def odometry_callback(self, msg: Odometry):
        # investigate data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.x
        z = msg.pose.pose.position.x

        or_x = msg.pose.pose.orientation.x
        or_y = msg.pose.pose.orientation.y
        or_z = msg.pose.pose.orientation.z

        # automatic closing after block
        with open(self.log_file, 'a+') as file:
            self.logger.info(f"File '{self.log_file}' opened successfully in mode '{self.mode}'.")
            file.write(f"{x},{y},{z},{or_x},{or_y},{or_z}\n")


# main method
def main(args=None):
    rclpy.init(args=args)

    imu = GazeboOdometryNode()
    rclpy.spin(imu)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


