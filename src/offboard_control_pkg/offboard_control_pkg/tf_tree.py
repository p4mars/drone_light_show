import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition 
import numpy as np 
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from pyproj import Transformer
from builtin_interfaces.msg import Time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleLocalPosition
from tf2_ros import StaticTransformBroadcaster
from pyproj import Transformer
from builtin_interfaces.msg import Time

class TF_Tree(Node):
    def __init__(self):
        super().__init__('tf_tree_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.map_lat = 47.397971057728974
        self.map_lon = 8.546163739800146
        self.map_alt = 0.0

        self.sent_transforms = {'drone1': False, 'drone2': False, 'drone3': False}
        self.static_transforms = []
        self.broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(VehicleLocalPosition,
                                 'px4_1/fmu/out/vehicle_local_position',
                                 self.drone1_callback, qos_profile)

        self.create_subscription(VehicleLocalPosition,
                                 'px4_2/fmu/out/vehicle_local_position',
                                 self.drone2_callback, qos_profile)

        self.create_subscription(VehicleLocalPosition,
                                 'px4_3/fmu/out/vehicle_local_position',
                                 self.drone3_callback, qos_profile)

    def gps_to_enu(self, lat, lon, alt):
        transformer = Transformer.from_crs(
            crs_from="epsg:4326",
            crs_to=f"+proj=aeqd +lat_0={self.map_lat} +lon_0={self.map_lon} +x_0=0 +y_0=0",
            always_xy=True
        )
        x, y = transformer.transform(lon, lat)
        z = alt - self.map_alt
        return x, y, z

    def publish_tf(self, drone_name, offset):
        t = TransformStamped()
        t.header.stamp = Time(sec=0, nanosec=0)
        t.header.frame_id = 'map'
        t.child_frame_id = f'{drone_name}_origin'
        t.transform.translation.x = offset[0]
        t.transform.translation.y = offset[1]
        t.transform.translation.z = offset[2]
        t.transform.rotation.w = 1.0

        self.static_transforms.append(t)
        self.get_logger().info(f'[TF] Staged: map → {drone_name}_origin @ ENU offset: {offset}')

    def publish_all_static_tfs(self):
        self.get_logger().info("[TF] Publishing all static transforms to /tf_static...")
        self.broadcaster.sendTransform(self.static_transforms)

    def check_if_ready(self):
        if all(self.sent_transforms.values()):
            self.publish_all_static_tfs()

    def drone1_callback(self, msg):
        if not self.sent_transforms['drone1']:
            self.get_logger().info("drone1_callback triggered")
            offset = self.gps_to_enu(msg.ref_lat, msg.ref_lon, msg.ref_alt)
            self.publish_tf("drone1", offset)
            self.sent_transforms['drone1'] = True
            self.check_if_ready()

    def drone2_callback(self, msg):
        if not self.sent_transforms['drone2']:
            self.get_logger().info("drone2_callback triggered")
            offset = self.gps_to_enu(msg.ref_lat, msg.ref_lon, msg.ref_alt)
            self.publish_tf("drone2", offset)
            self.sent_transforms['drone2'] = True
            self.check_if_ready()

    def drone3_callback(self, msg):
        if not self.sent_transforms['drone3']:
            self.get_logger().info("drone3_callback triggered")
            offset = self.gps_to_enu(msg.ref_lat, msg.ref_lon, msg.ref_alt)
            self.publish_tf("drone3", offset)
            self.sent_transforms['drone3'] = True
            self.check_if_ready()


def main(args=None) -> None:
    try:
        print('Starting offboard control node...')
        rclpy.init(args=args)
        offboard_control = TF_Tree()
        rclpy.spin(offboard_control)
        offboard_control.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"Exception occurred in Drone_One Node: {e}")
        rclpy.shutdown()