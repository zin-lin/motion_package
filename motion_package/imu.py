import rclpy
from rclpy.node import Node
from annex_msgs.msg import Vcu2ai, Ai2vcu
from sensor_msgs.msg import Imu
from ament_index_python.packages import get_package_prefix
import os
import time
import datetime

PKG_NAME = "motion_package"
LOG = "data_logs"
FILE = f"imu{datetime.datetime.now()}.csv"


class GazeboIMUNode(Node):
    def __init__(self):
        super().__init__('IMU_node')
        # initiate vars
        self.logger = self.get_logger()
        # initiate log file
        current_directory = os.path.dirname(os.path.abspath(__file__))
        self.log_file = os.path.join(current_directory,LOG, FILE)
        self.mode = "a+" if os.path.isfile(self.log_file) else "w+"
        self.time = time.time()
        self._sub_pub()

    # subscription and publishing
    def _sub_pub(self):
        self.create_subscription(Imu, "model/adsmt/imu", self.imu_callback, 10)

    # IMU callback: called everytime the IMU data is published
    def imu_callback(self, msg: Imu):
        # investigate data
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        rot_x = msg.angular_velocity.x
        rot_y = msg.angular_velocity.y
        rot_z = msg.angular_velocity.z

        # time elapsed
        current_time = time.time()
        lapsed = current_time - self.time

        # automatic closing after block
        with open(self.log_file, 'a+') as file:
            self.logger.info(f"File '{self.log_file}' opened successfully in mode '{self.mode}'.")
            file.write(f"{acc_x},{acc_y},{acc_z},{rot_x},{rot_y},{rot_z},{lapsed}\n")



# main method
def main(args=None):
    rclpy.init(args=args)

    imu = GazeboIMUNode()

    rclpy.spin(imu)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


