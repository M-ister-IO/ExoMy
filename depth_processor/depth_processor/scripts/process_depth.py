import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header
import numpy as np


class DepthProcessor(Node):
    def __init__(self):
        super().__init__("depth_processor")
        self.points = None
        self.depth_sub = self.create_subscription(Float32MultiArray, "raw_depth", self.depth_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud2, "point_cloud", 10)
        self.header = Header()
        self.header.frame_id = "sensor_frame"
        self.width_=240
        self.height_=180
        self.fx = 192.92
        self.fy = 191.25

        self.get_logger().info("DepthProcessor initialized")
        self.get_logger().info("Subscriptions and publisher created")

    def depth_callback(self, msg: Float32MultiArray):
        self.get_logger().info("Depth data received")
        self.depth_data = np.array(msg.data) / 1000  # Convert to meters
        self.process_point_cloud()


    def process_point_cloud(self):
        self.get_logger().info("Processing point cloud")
        if hasattr(self, 'depth_data'):


            z = self.depth_data
            z[z <= 0] = np.nan  # Handling invalid depth values
            z[z > 1.5] = np.nan  # Handling invalid depth values
            z = z.reshape((180, 240))  # Ensure it matches the shape of u and v


            # Calculate x and y coordinates
            u = np.arange(self.width_)
            v = np.arange(self.height_)
            u, v = np.meshgrid(u, v)

            # Assuming self.fx, self.fy, self.width_, and self.height_ are known

            x = (u - self.width_ / 2) * z / self.fx
            y = (v - self.height_ / 2) * z / self.fy

            # Combined point cloud
            points = np.stack((x, y, z), axis=-1)
            points = points[~np.isnan(points).any(axis=-1)]  # Filter invalid points

            pc2_msg_ = point_cloud2.create_cloud_xyz32(self.header, points)
            self.get_logger().info("Point cloud published")
            self.publisher_.publish(pc2_msg_)
            


def main(args=None):
    rclpy.init(args=args)
    point_cloud_processor = DepthProcessor()
    rclpy.spin(point_cloud_processor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
