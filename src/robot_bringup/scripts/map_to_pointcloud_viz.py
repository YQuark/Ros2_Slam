#!/usr/bin/env python3

import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class MapToPointCloudViz(Node):
    def __init__(self):
        super().__init__('map_to_pointcloud_viz')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('points_topic', '/map_points')
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('free_threshold', 20)
        self.declare_parameter('publish_free_cells', True)
        self.declare_parameter('free_cell_stride', 4)
        self.declare_parameter('z_height', 0.02)

        map_topic = self.get_parameter('map_topic').value
        points_topic = self.get_parameter('points_topic').value
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.publish_free_cells = bool(self.get_parameter('publish_free_cells').value)
        self.free_cell_stride = max(1, int(self.get_parameter('free_cell_stride').value))
        self.z_height = float(self.get_parameter('z_height').value)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        points_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_cb, map_qos
        )
        self.pub = self.create_publisher(PointCloud2, points_topic, points_qos)

        self.get_logger().info(
            f'map_to_pointcloud_viz started: {map_topic} -> {points_topic}, threshold={self.occupied_threshold}'
        )

    def _map_cb(self, msg: OccupancyGrid):
        info = msg.info
        width = info.width
        height = info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        point_step = 16  # x,y,z,intensity as float32
        data = bytearray()

        for idx, occ in enumerate(msg.data):
            if occ < 0:
                continue

            is_occupied = occ >= self.occupied_threshold
            is_free = self.publish_free_cells and occ <= self.free_threshold
            if not is_occupied and not is_free:
                continue

            y_cell = idx // width
            x_cell = idx % width
            x = ox + (x_cell + 0.5) * res
            y = oy + (y_cell + 0.5) * res
            if is_free:
                # Downsample free cells so RViz stays responsive on large explored maps.
                if (x_cell % self.free_cell_stride) != 0 or (y_cell % self.free_cell_stride) != 0:
                    continue
                intensity = 20.0
            else:
                intensity = 100.0

            data.extend(struct.pack('<ffff', x, y, self.z_height, intensity))

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id if msg.header.frame_id else 'map'

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(data) // point_step
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = point_step
        cloud.row_step = point_step * cloud.width
        cloud.is_dense = True
        cloud.data = bytes(data)

        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = MapToPointCloudViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
