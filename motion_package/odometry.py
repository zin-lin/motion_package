import time
import datetime
import rclpy
from rclpy.node import Node
from annex_msgs.msg import Vcu2ai, Ai2vcu
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_prefix
import os

PKG_NAME = "motion_package"
LOG = "data_logs"
FILE = f"odometry{datetime.datetime.now()}.csv"


class GazeboOdometryNode(Node):
    def __init__(self):
        super().__init__('Odometry_node')
        # initiate vars
        self.logger = self.get_logger()
        # initiate log file
        current_directory = os.path.dirname(os.path.abspath(__file__))
        self.log_file = os.path.join(current_directory, LOG, FILE)
        self.mode = "a+" if os.path.isfile(self.log_file) else "w+"
        self.time = time.time()
        self._sub_pub()

    # subscription and publishing
    def _sub_pub(self):
        self.create_subscription(Odometry, "model/adsmt/odometry", self.odometry_callback, 10)

    # IMU callback: called everytime the IMU data is published
    def odometry_callback(self, msg: Odometry):
        # investigate data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        or_x = msg.pose.pose.orientation.x
        or_y = msg.pose.pose.orientation.y
        or_z = msg.pose.pose.orientation.z

        # time elapsed
        current_time = time.time()
        lapsed = current_time - self.time

        # automatic closing after block
        with open(self.log_file, 'a+') as file:
            self.logger.info(f"File '{self.log_file}' opened successfully in mode '{self.mode}'.")
            file.write(f"{x},{y},{z},{or_x},{or_y},{or_z},{lapsed}\n")


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


