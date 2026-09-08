#!/usr/bin/env python3
# Copyright 2026 OpenArm contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Tabletop object perception for the bimanual OpenArm simulation.
#
# Segments the simulated D435 depth point cloud into tabletop object clusters
# and publishes vision_msgs/Detection3DArray with the cluster centroid and
# axis-aligned bounding box in the robot `world` frame, plus an RViz
# MarkerArray for inspection.

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from shape_msgs.msg import SolidPrimitive  # noqa: F401  (used via vision_msgs docs)
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

DETECTION3D_ARRAY = 'vision_msgs/msg/Detection3DArray'


def quaternion_matrix(q):
    """Quaternion [x, y, z, w] -> 4x4 homogeneous rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0.0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0.0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0]])


def cloud_to_xyz(cloud: PointCloud2) -> np.ndarray:
    """Extract an (N, 3) float array of valid points from a PointCloud2."""
    fields = {f.name: f for f in cloud.fields}
    offsets = np.array([fields['x'].offset, fields['y'].offset, fields['z'].offset],
                       dtype=np.int32)
    dt = np.dtype({'names': ['x', 'y', 'z'],
                   'formats': [np.float32] * 3,
                   'offsets': offsets,
                   'itemsize': cloud.point_step})
    pts = np.frombuffer(cloud.data, dtype=dt)
    valid = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
    pts = pts[valid]
    return np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(np.float64)


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('bimanual_perception')

        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('cloud_topic', '/camera/depth/color/points')
        self.declare_parameter('processing_rate', 2.0)
        # Work region in the robot world frame (table top sits at ~0.105).
        self.declare_parameter('crop_min', [-0.45, -0.52, -0.06])
        self.declare_parameter('crop_max', [0.45, -0.16, 0.30])
        self.declare_parameter('voxel_size', 0.012)
        self.declare_parameter('cluster_cell', 0.025)
        self.declare_parameter('min_cluster_points', 60)
        self.declare_parameter('plane_margin', 0.012)

        gp = self.get_parameter
        self.target_frame = gp('target_frame').value
        self.crop_min = np.array(gp('crop_min').value)
        self.crop_max = np.array(gp('crop_max').value)
        self.voxel_size = gp('voxel_size').value
        self.cluster_cell = gp('cluster_cell').value
        self.min_points = gp('min_cluster_points').value
        self.plane_margin = gp('plane_margin').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_sub = self.create_subscription(
            PointCloud2, gp('cloud_topic').value, self.cloud_cb, sensor_qos)

        self.det_pub = self.create_publisher(Detection3DArray,
                                             '/bimanual_perception/objects', 10)
        self.marker_pub = self.create_publisher(MarkerArray,
                                                '/bimanual_perception/markers', 10)

        self.latest_cloud = None
        self.timer = self.create_timer(1.0 / gp('processing_rate').value, self.process)

    def cloud_cb(self, cloud: PointCloud2):
        self.latest_cloud = cloud

    def process(self):
        cloud = self.latest_cloud
        if cloud is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, cloud.header.frame_id, cloud.header.stamp,
                timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'tf lookup failed: {e}', throttle_duration_sec=5.0)
            return

        pts = cloud_to_xyz(cloud)
        if pts.shape[0] < 100:
            self.get_logger().warn('empty cloud', throttle_duration_sec=5.0)
            return

        # Transform camera -> world (rotation then translation).
        q = tf.transform.rotation
        rot = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z])
        pts = pts @ rot.T + t

        # Crop to the tabletop work region.
        keep = np.all((pts >= self.crop_min) & (pts <= self.crop_max), axis=1)
        pts = pts[keep]
        if pts.shape[0] < 100:
            return

        # Voxel downsample.
        voxel = self.voxel_size
        keys = np.floor(pts / voxel).astype(np.int64)
        _, unique_idx = np.unique(keys, axis=0, return_index=True)
        pts = pts[np.sort(unique_idx)]

        # Remove the dominant table plane via a z-histogram.
        z_lo, z_hi = self.crop_min[2], self.crop_max[2]
        bins = np.arange(z_lo, z_hi + 0.005, 0.01)
        hist, edges = np.histogram(pts[:, 2], bins=bins)
        if hist.max() < 50:
            return
        plane_z = (edges[hist.argmax()] + edges[hist.argmax() + 1]) / 2.0
        pts = pts[pts[:, 2] > plane_z + self.plane_margin]
        if pts.shape[0] < self.min_points:
            return

        # Grid-based connected-component clustering.
        cell = self.cluster_cell
        keys = np.floor(pts / cell).astype(np.int64)
        key_to_idx = {}
        for i, k in enumerate(map(tuple, keys)):
            key_to_idx.setdefault(k, []).append(i)

        seen = np.zeros(pts.shape[0], dtype=bool)
        clusters = []
        for start in range(pts.shape[0]):
            if seen[start]:
                continue
            stack = [tuple(keys[start])]
            members = []
            seen[start] = True
            while stack:
                k = stack.pop()
                for i in key_to_idx.get(k, ()):
                    if not seen[i]:
                        seen[i] = True
                        members.append(i)
                        stack.append(tuple(keys[i]))
                for dk0 in (-1, 0, 1):
                    for dk1 in (-1, 0, 1):
                        for dk2 in (-1, 0, 1):
                            nk = (k[0] + dk0, k[1] + dk1, k[2] + dk2)
                            for i in key_to_idx.get(nk, ()):
                                if not seen[i]:
                                    seen[i] = True
                                    members.append(i)
                                    stack.append(nk)
            if len(members) >= self.min_points:
                clusters.append(pts[members])

        # Sort big first for stable ids.
        clusters.sort(key=lambda c: -c.shape[0])
        self.publish_detections(clusters, cloud.header.stamp)

    def publish_detections(self, clusters, stamp):
        detections = Detection3DArray()
        detections.header.stamp = stamp
        detections.header.frame_id = self.target_frame

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        palette = [(0.9, 0.2, 0.2), (0.2, 0.7, 0.9), (0.3, 0.85, 0.3),
                   (0.95, 0.7, 0.1), (0.8, 0.3, 0.9), (0.9, 0.9, 0.2),
                   (0.4, 0.5, 0.95)]

        for n, cluster in enumerate(clusters):
            center = cluster.mean(axis=0)
            size = cluster.max(axis=0) - cluster.min(axis=0)

            det = Detection3D()
            det.header.stamp = stamp
            det.header.frame_id = self.target_frame
            det.id = f'obj_{n}'
            det.bbox.center.position = Point(x=float(center[0]), y=float(center[1]),
                                             z=float(center[2]))
            det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = map(float, size)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = f'obj_{n}'
            hyp.hypothesis.score = float(min(1.0, cluster.shape[0] / 500.0))
            det.results.append(hyp)
            detections.detections.append(det)

            color = palette[n % len(palette)]
            mk = Marker()
            mk.header.stamp = stamp
            mk.header.frame_id = self.target_frame
            mk.ns = 'objects'
            mk.id = n
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position = det.bbox.center.position
            mk.pose.orientation.w = 1.0
            mk.scale.x, mk.scale.y, mk.scale.z = (max(s, 0.01) for s in size)
            mk.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.45)
            markers.markers.append(mk)

            txt = Marker()
            txt.header = mk.header
            txt.ns = 'labels'
            txt.id = n
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position = Point(x=float(center[0]), y=float(center[1]),
                                      z=float(center[2] + size[2] / 2 + 0.04))
            txt.scale.z = 0.03
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            txt.text = f'obj_{n} {size[0]*100:.0f}x{size[1]*100:.0f}x{size[2]*100:.0f}cm'
            markers.markers.append(txt)

        self.det_pub.publish(detections)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
