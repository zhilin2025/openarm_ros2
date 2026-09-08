#!/usr/bin/env python3
# Copyright 2026 OpenArm contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Bimanual cooperative carrying coordinator for the OpenArm v11 simulation.
#
# For every object in the configured sequence the node:
#   1. teleports the object to its start pose and perceives it (head-camera
#      point-cloud clusters, or gazebo ground truth with use_ground_truth),
#   2. decides the bimanual dual top-press grasp poses for both arms from
#      the object's size,
#   3. solves IK itself (scipy bounded least squares, multi-start - the KDL
#      solver inside move_group is unreliable for the redundant 7-DoF arms)
#      and plans JOINT-SPACE goals with MoveIt OMPL (collision-checked),
#   4. executes the two arms' trajectories time-synchronized; the short
#      grasp/lift/carry motions use direct joint interpolation,
#   5. welds the object to the left hand via the /bimanual_grasp plugin,
#      lifts, carries in short Cartesian segments, releases over the
#      destination, retreats and returns home,
#   6. verifies the final object pose and parks the object behind the work
#      zone so the next test starts from a clean table.

import json
import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as RosDuration
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Point, Pose, Quaternion
from gazebo_msgs.srv import GetEntityState, SetEntityState
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint,
                             MotionPlanRequest)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray

LEFT_ARM_JOINTS = [f'openarm_left_joint{i}' for i in range(1, 8)]
RIGHT_ARM_JOINTS = [f'openarm_right_joint{i}' for i in range(1, 8)]

# Hand frame geometry (v11, both hands): fingers extend along -Z, fingertip
# faces 0.206 m in front of the hand origin. The grasp presses the fingertip
# faces vertically onto the object top.
FINGER_END = 0.206
PAD_HALF_WIDTH = 0.0305   # fingertip face half width along hand X

XHAT = np.array([1.0, 0.0, 0.0])
YHAT = np.array([0.0, 1.0, 0.0])
ZHAT = np.array([0.0, 0.0, 1.0])

TABLE_TOP = 0.105          # table surface in the robot world frame

# v11 arm chain (both arms share the structure; x is mirrored for the right
# arm): translation from the previous link, rotation axis, joint limits.
ARM_CHAIN = [
    ((0.02955, 0, 0), (-1, 0, 0), -1.65, 3.21),
    ((0.067, 0.0229, 0), (0, -1, 0), -0.36, 2.85),
    ((0, -0.0301, -0.07825), (0, 0, -1), -1.56, 1.56),
    ((0, 0, -0.1625), (-1, 0, 0), 0.0, 2.43),
    ((-0.0005, 0, -0.0995), (0, 0, -1), -1.56, 1.56),
    ((0, 0, -0.1259), (0, -1, 0), -0.6, 0.6),
    ((0.0199, 0, 0), (-1, 0, 0), -1.56, 1.56),
]
ARM_BASE = {          # shoulder (openarm_*_link0) in the robot world frame;
                      # the URDF world link sits 0.38 m above the body root
    'left_arm': np.array([0.058, 0.0, 0.698]),
    'right_arm': np.array([-0.059, 0.0, 0.698]),
}
RIGHT_ARM_LIMITS = [(-1.65, 3.21), (-2.85, 0.36), (-1.56, 1.56),
                    (0.0, 2.43), (-1.56, 1.56), (-0.6, 0.6),
                    (-1.56, 1.56)]

# name -> {size, grasp, round, press_offset, start, dest, park}
# All poses in the robot world frame (gazebo z minus the 0.38 spawn offset).
OBJECT_TEMPLATES = {
    'water_bottle': {
        'size': [0.076, 0.076, 0.16], 'grasp': [0.076, 0.076, 0.15],
        'round': True, 'press_offset': 0.035,
        'start': [0.0, -0.30, 0.18], 'dest': [-0.14, -0.30, 0.18],
        'park': [-0.24, -0.75, 0.18]},
    'food_can': {
        'size': [0.07, 0.07, 0.13], 'grasp': [0.07, 0.07, 0.13],
        'round': True, 'press_offset': 0.035,
        'start': [0.0, -0.30, 0.17], 'dest': [-0.14, -0.30, 0.17],
        'park': [-0.12, -0.75, 0.17]},
    'cereal_box': {
        'size': [0.11, 0.05, 0.16], 'grasp': [0.10, 0.045, 0.15],
        'press_offset': 0.035,
        'start': [0.0, -0.31, 0.18], 'dest': [-0.12, -0.30, 0.18],
        'park': [0.0, -0.75, 0.18]},
    'coffee_mug': {
        'size': [0.09, 0.19, 0.14], 'grasp': [0.09, 0.09, 0.14],
        'round': True, 'press_offset': 0.041,
        'start': [0.02, -0.30, 0.175], 'dest': [-0.16, -0.30, 0.175],
        'park': [0.12, -0.75, 0.175]},
    'apple': {
        'size': [0.094, 0.094, 0.115], 'grasp': [0.094, 0.094, 0.115],
        'round': True, 'press_offset': 0.043,
        'start': [0.0, -0.30, 0.152], 'dest': [-0.16, -0.30, 0.152],
        'park': [-0.30, -0.75, 0.152]},
    'tissue_box': {
        'size': [0.17, 0.12, 0.095], 'grasp': [0.16, 0.12, 0.095],
        'press_offset': 0.060,
        'start': [0.02, -0.30, 0.1525], 'dest': [-0.14, -0.30, 0.1525],
        'park': [0.24, -0.75, 0.1525]},
    'storage_box': {
        'size': [0.26, 0.20, 0.13], 'grasp': [0.26, 0.20, 0.13],
        'press_offset': 0.110,
        'start': [0.0, -0.31, 0.17], 'dest': [-0.10, -0.31, 0.17],
        'park': [0.40, -0.75, 0.17]},
}


def rot(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def matrix_to_quat(m):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw, qx, qy, qz = 0.25 * s, (m[2, 1] - m[1, 2]) / s, \
            (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 0] and m[0, 0] > m[2, 0]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 0] - m[2, 0]) * 2
        qw, qx, qy, qz = (m[2, 1] - m[1, 2]) / s, 0.25 * s, \
            (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 0] > m[2, 0]:
        s = math.sqrt(1.0 + m[1, 0] - m[0, 0] - m[2, 0]) * 2
        qw, qx, qy, qz = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, \
            0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 0] - m[0, 0] - m[1, 0]) * 2
        qw, qx, qy, qz = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, \
            (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([qx, qy, qz, qw])
    return q / np.linalg.norm(q)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class ArmKinematics:
    """Numerical FK + bounded least-squares IK for one v11 arm chain."""

    def __init__(self, group):
        self.group = group
        mirror = 1.0 if group == 'left_arm' else -1.0
        self.chain = []
        for off, ax, lo, hi in ARM_CHAIN:
            off = np.array(off, float)
            off[0] *= mirror
            self.chain.append((off, np.array(ax, float), lo, hi))
        if group == 'right_arm':
            self.chain = [(c[0], c[1], lo, hi)
                          for c, (lo, hi) in zip(self.chain, RIGHT_ARM_LIMITS)]
        self.base = ARM_BASE[group]
        self.lo = np.array([c[2] for c in self.chain])
        self.hi = np.array([c[3] for c in self.chain])
        self.bank_pos = np.zeros((0, 3))
        self.bank_q = np.zeros((0, 7))

    def fk(self, q):
        T = np.eye(4)
        T[:3, 3] = self.base      # the chain starts at the shoulder (link0)
        for (off, ax, lo, hi), qi in zip(self.chain, q):
            R = T[:3, :3]
            t = T[:3, 3] + R @ off
            R = R @ rot(ax, qi)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t
        return T

    def fk_origins(self, q):
        """FK returning the world positions of every joint origin too."""
        T = np.eye(4)
        T[:3, 3] = self.base
        pts = []
        for (off, ax, lo, hi), qi in zip(self.chain, q):
            R = T[:3, :3]
            t = T[:3, 3] + R @ off
            R = R @ rot(ax, qi)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t
            pts.append(T[:3, 3].copy())
        return T, pts

    def build_seed_bank(self, n=60000):
        """Collect joint configurations whose hand points roughly straight
        down (hand Z up) in the bimanual work zone, with the elbow kept out
        of the table box; used as IK seeds."""
        rng = np.random.default_rng(11)
        bank_pos, bank_q = [], []
        lo, hi = self.lo, self.hi
        for _ in range(n):
            q = rng.uniform(lo, hi)
            T, pts = self.fk_origins(q)
            p = T[:3, 3]
            if T[2, 2] > 0.85 and p[2] > 0.2 and -0.65 < p[1] < -0.05 \
                    and abs(p[0]) < 0.4:
                if any(pt[1] < -0.18 and pt[2] < TABLE_TOP + 0.02
                       for pt in pts[2:6]):
                    continue
                bank_pos.append(p)
                bank_q.append(q)
        self.bank_pos = np.array(bank_pos)
        self.bank_q = np.array(bank_q)

    def nearest_bank_seed(self, target_pos):
        if len(self.bank_pos) == 0:
            return None
        d = np.linalg.norm(self.bank_pos - target_pos, axis=1)
        return self.bank_q[int(np.argmin(d))]

    def ik(self, target_pos, target_R, seeds=(), restarts=12, iterations=200):
        """Bounded least-squares IK (scipy), multi-start."""
        from scipy.optimize import least_squares
        lo, hi = self.lo, self.hi

        def fun(q):
            T = self.fk(q)
            e_pos = (T[:3, 3] - target_pos) * 3.0
            Rc = T[:3, :3].T @ target_R
            angle = math.acos(max(-1.0, min(1.0, (np.trace(Rc) - 1) / 2)))
            if angle < 1e-6:
                e_orn = np.zeros(3)
            else:
                v = np.array([Rc[2, 1] - Rc[1, 2], Rc[0, 2] - Rc[2, 0],
                              Rc[1, 0] - Rc[0, 1]])
                nv = np.linalg.norm(v)
                e_orn_local = v / nv * angle if nv > 1e-9 else np.zeros(3)
                e_orn = T[:3, :3] @ e_orn_local
            return np.concatenate([e_pos, e_orn])

        rng = np.random.default_rng()
        starts = [np.asarray(s, float) for s in seeds]
        starts.append(np.zeros(7))
        bank_seed = self.nearest_bank_seed(target_pos)
        if bank_seed is not None:
            starts.append(bank_seed)
        for _ in range(restarts):
            q0 = rng.uniform(lo, hi)
            q0[3] = rng.uniform(0.4, 1.8)
            q0[1] = rng.uniform(0.0, 1.2)
            starts.append(np.clip(q0, lo, hi))
        for x0 in starts:
            x0 = np.clip(x0, lo, hi)
            res = least_squares(fun, x0, bounds=(lo, hi), max_nfev=iterations)
            T, pts = self.fk_origins(res.x)
            if np.linalg.norm(T[:3, 3] - target_pos) < 0.006:
                Rc = T[:3, :3].T @ target_R
                ang = math.acos(max(-1.0, min(1.0, (np.trace(Rc) - 1) / 2)))
                if ang >= 0.2:
                    continue
                # keep the elbow/wrist links out of the table box
                bad = any(pt[1] < -0.18 and pt[2] < TABLE_TOP + 0.02
                          for pt in pts[2:6])
                if not bad:
                    return res.x
        return None


def pose_from_zy(z_axis, position, sep_axis=None):
    """Hand pose whose Z axis is z_axis (fingers extend along -Z) and whose
    finger separation axis is `sep_axis` (world +Y by default)."""
    z_axis = np.asarray(z_axis, float)
    z_axis = z_axis / np.linalg.norm(z_axis)
    sep = YHAT if sep_axis is None else np.asarray(sep_axis, float)
    y_axis = sep - sep.dot(z_axis) * z_axis
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    q = matrix_to_quat(np.column_stack([x_axis, y_axis, z_axis]))
    return Pose(position=Point(x=float(position[0]), y=float(position[1]),
                               z=float(position[2])),
                orientation=Quaternion(x=float(q[0]), y=float(q[1]),
                                       z=float(q[2]), w=float(q[3])))


def shifted(pose, delta):
    q = Pose()
    q.position = Point(x=pose.position.x + delta[0],
                       y=pose.position.y + delta[1],
                       z=pose.position.z + delta[2])
    q.orientation = pose.orientation
    return q


def ros_time(seconds):
    return RosDuration(seconds=int(seconds),
                       nanoseconds=int(round((seconds % 1.0) * 1e9))).to_msg()


def quat_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0.0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0.0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0]])


class CarryNode(Node):

    def __init__(self):
        super().__init__('bimanual_carry')

        self.declare_parameter('object_sequence',
                               ['water_bottle', 'food_can', 'cereal_box',
                                'coffee_mug', 'apple', 'tissue_box',
                                'storage_box'])
        self.declare_parameter('use_ground_truth', False)
        self.declare_parameter('planning_time', 4.0)
        self.declare_parameter('velocity_scaling', 0.15)
        self.declare_parameter('approach_dist', 0.04)
        self.declare_parameter('lift_height', 0.06)
        self.declare_parameter('gazebo_tf_z_offset', 0.38)
        self.declare_parameter('templates_file', '')

        self.seq = self.get_parameter('object_sequence').value
        self.templates = {k: dict(v) for k, v in OBJECT_TEMPLATES.items()}
        templates_file = self.get_parameter('templates_file').value
        if templates_file:
            import yaml
            with open(templates_file, 'r', encoding='utf-8') as f:
                self.templates.update(yaml.safe_load(f) or {})
        self.use_gt = self.get_parameter('use_ground_truth').value
        self.planning_time = self.get_parameter('planning_time').value
        self.vel_scaling = self.get_parameter('velocity_scaling').value
        self.approach_dist = self.get_parameter('approach_dist').value
        self.lift_height = self.get_parameter('lift_height').value
        self.gz_z = self.get_parameter('gazebo_tf_z_offset').value

        self.kin = {
            'left_arm': ArmKinematics('left_arm'),
            'right_arm': ArmKinematics('right_arm'),
        }
        for k in self.kin.values():
            k.build_seed_bank()
        self.last_q = {'left_arm': None, 'right_arm': None}

        self.grasp_status = {}
        self.latest_detections = None
        self.latest_joint_state = None

        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.scene_client = self.create_client(ApplyPlanningScene,
                                               '/apply_planning_scene')
        self.scene_get_client = self.create_client(GetPlanningScene,
                                                   '/get_planning_scene')
        self.state_get_client = self.create_client(GetEntityState,
                                                   '/gazebo/get_entity_state')
        self.state_set_client = self.create_client(SetEntityState,
                                                   '/gazebo/set_entity_state')
        self.cmd_pub = self.create_publisher(String, '/bimanual_grasp/grasp_cmd', 10)
        self.marker_pub = self.create_publisher(MarkerArray,
                                                '/bimanual_perception/grasp_plan', 10)
        self.create_subscription(String, '/bimanual_grasp/grasp_status',
                                 self.status_cb, 10)
        self.create_subscription(Detection3DArray, '/bimanual_perception/objects',
                                 self.detections_cb, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self.joint_state_cb, 20)

        self.traj_clients = {
            'left_arm': ActionClient(self, FollowJointTrajectory,
                                     '/left_joint_trajectory_controller/'
                                     'follow_joint_trajectory'),
            'right_arm': ActionClient(self, FollowJointTrajectory,
                                      '/right_joint_trajectory_controller/'
                                      'follow_joint_trajectory'),
        }
        self.gripper_clients = {
            'left_arm': ActionClient(self, GripperCommand,
                                     '/left_gripper_controller/gripper_cmd'),
            'right_arm': ActionClient(self, GripperCommand,
                                      '/right_gripper_controller/gripper_cmd'),
        }
        self.get_logger().info('waiting for /plan_kinematic_path ...')
        self.plan_client.wait_for_service(timeout_sec=180.0)
        self.get_logger().info('move_group ready')

    # ------------------------------------------------------------------
    # callbacks / small helpers
    # ------------------------------------------------------------------
    def status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.grasp_status[data['id']] = data
        except Exception:
            pass

    def detections_cb(self, msg):
        self.latest_detections = msg

    def joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def wait_status(self, hand_id, event, timeout=6.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            info = self.grasp_status.get(hand_id)
            if info and info.get('event') == event:
                return info
            rclpy.spin_once(self, timeout_sec=0.05)
        return None

    def send_grasp_cmd(self, action, hand_id, link='', object_name=''):
        self.grasp_status.pop(hand_id, None)
        msg = String()
        msg.data = json.dumps({'action': action, 'id': hand_id, 'link': link,
                               'object': object_name})
        self.cmd_pub.publish(msg)

    def gazebo_pose(self, name):
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = 'world'
        fut = self.state_get_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None or not fut.result().success:
            return None
        p = fut.result().state.pose.position
        return (p.x, p.y, p.z)

    def teleport(self, name, robot_frame_xyz, yaw=0.0):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(robot_frame_xyz[0])
        req.state.pose.position.y = float(robot_frame_xyz[1])
        req.state.pose.position.z = float(robot_frame_xyz[2] + self.gz_z)
        qz = [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]
        req.state.pose.orientation.x = float(qz[0])
        req.state.pose.orientation.y = float(qz[1])
        req.state.pose.orientation.z = float(qz[2])
        req.state.pose.orientation.w = float(qz[3])
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0
        req.state.reference_frame = 'world'
        for _ in range(2):
            fut = self.state_set_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            time.sleep(0.8)  # let the object settle
            p = self.gazebo_pose(name)
            if p is None:
                return False
            err = np.linalg.norm(np.array(p[:3]) - np.array(
                [robot_frame_xyz[0], robot_frame_xyz[1],
                 robot_frame_xyz[2] + self.gz_z]))
            if err <= 0.02:
                return True
            self.get_logger().warn(
                f'teleport readback mismatch ({err * 100:.1f} cm), retrying')
        return False

    # ------------------------------------------------------------------
    # grasp geometry: bimanual dual top-press
    # ------------------------------------------------------------------
    def grasp_spec(self, center, tpl):
        """Bimanual side grasp: the left hand presses the object's +X face
        and the right hand its -X face, both arms approaching diagonally
        from above their own side (45 deg). The arms stay clearly on the two
        sides of the object instead of stacking on top of it."""
        gx, gy, gz = tpl['grasp']
        x, y, z = center
        spec = {'mode': 'dual_side', 'center': np.array(center, float),
                'hands': {}}
        s2c, c2s = 0.70710678, 0.70710678
        for side, sign in (('left', +1.0), ('right', -1.0)):
            # contact point on the upper part of the +/-X face
            contact = np.array([x + sign * (gx / 2.0 + 0.005), y,
                                z + gz * 0.6])
            # hand Z axis points outward+up at 45 deg (fingers reach the
            # face from above their own side)
            z_hand = np.array([sign * s2c, 0.0, c2s])
            palm = contact + FINGER_END * z_hand
            grasp = pose_from_zy(z_hand, palm, sep_axis=YHAT)
            pre = pose_from_zy(z_hand,
                               palm + self.approach_dist * z_hand,
                               sep_axis=YHAT)
            spec['hands'][side] = {'grasp': grasp, 'pre_grasp': pre,
                                   'q_open': 0.02, 'q_close': 0.02,
                                   'link': f'openarm_{side}_hand'}
        spec['dest_center'] = np.array(tpl['dest'], float)
        return spec

    def translated_spec(self, spec, delta):
        out = {'mode': spec['mode'], 'center': spec['center'] + delta,
               'dest_center': spec['dest_center'], 'hands': {}}
        for side, h in spec['hands'].items():
            nh = dict(h)
            nh['grasp'] = shifted(h['grasp'], delta)
            nh['pre_grasp'] = shifted(h['pre_grasp'], delta)
            out['hands'][side] = nh
        return out

    # ------------------------------------------------------------------
    # own IK + motion generation
    # ------------------------------------------------------------------
    def current_q(self, group):
        js = self.latest_joint_state
        if js is None:
            return np.zeros(7)
        prefix = 'openarm_left' if group == 'left_arm' else 'openarm_right'
        try:
            return np.array([js.position[js.name.index(f'{prefix}_joint{i}')]
                             for i in range(1, 8)])
        except ValueError:
            return np.zeros(7)

    def solve_ik(self, group, pose):
        R = quat_to_R([pose.orientation.x, pose.orientation.y,
                       pose.orientation.z, pose.orientation.w])
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        seeds = [] if self.last_q[group] is None else [self.last_q[group]]
        q = self.kin[group].ik(pos, R, seeds=seeds)
        if q is None:
            return None
        self.last_q[group] = q
        return q

    def make_joint_traj(self, group, q_to, duration):
        q_to = np.asarray(q_to, float)
        q_from = self.current_q(group)
        joints = LEFT_ARM_JOINTS if group == 'left_arm' else RIGHT_ARM_JOINTS
        jt = JointTrajectory()
        jt.joint_names = joints
        n = 25
        for i in range(n + 1):
            f = i / n
            f = f * f * (3 - 2 * f)          # smoothstep
            pt = JointTrajectoryPoint()
            pt.positions = list(q_from * (1 - f) + q_to * f)
            pt.time_from_start = ros_time(duration * f)
            jt.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        return goal

    def execute_joint_goals(self, ql, qr, label, duration=2.5):
        """Send direct joint-space goals (no MoveIt) to both arms."""
        goals = [('left_arm', self.make_joint_traj('left_arm', ql, duration)),
                 ('right_arm', self.make_joint_traj('right_arm', qr, duration))]
        ok = True
        for group, goal in goals:
            fut = self.traj_clients[group].send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(f'{group} rejected {label}')
                ok = False
                continue
            res_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut,
                                             timeout_sec=duration + 30.0)
            result = res_fut.result()
            code = result.result.error_code if result else -1
            if code != 0:
                self.get_logger().error(f'{group} failed {label} (code {code})')
                ok = False
        return ok

    def interp_phase(self, targets, duration=1.2):
        """targets: {'left_arm': Pose, 'right_arm': Pose}. IK for both arms
        (seeded by the previous phase) followed by a direct joint-space
        interpolation executed simultaneously on both controllers."""
        qs = {}
        for group, pose in targets.items():
            q = self.solve_ik(group, pose)
            if q is None:
                self.get_logger().error(f'{group}: IK failed for target')
                return False
            qs[group] = q
        futures = [(g, self.traj_clients[g].send_goal_async(
            self.make_joint_traj(g, qs[g], duration))) for g in qs]
        ok = True
        for group, fut in futures:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(f'{group} rejected interp goal')
                ok = False
                continue
            res_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut,
                                             timeout_sec=duration + 20.0)
            result = res_fut.result()
            code = result.result.error_code if result else -1
            if code != 0:
                self.get_logger().error(f'{group} interp failed ({code})')
                ok = False
        return ok

    def plan_joints(self, group, positions):
        """OMPL joint-space plan (collision-checked against the scene)."""
        joints = LEFT_ARM_JOINTS if group == 'left_arm' else RIGHT_ARM_JOINTS
        req = MotionPlanRequest()
        req.group_name = group
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnectkConfigDefault'
        req.allowed_planning_time = self.planning_time
        req.num_planning_attempts = 4
        req.max_velocity_scaling_factor = self.vel_scaling
        req.max_acceleration_scaling_factor = self.vel_scaling
        c = Constraints()
        k = self.kin[group]
        for i, (name, value) in enumerate(zip(joints, positions)):
            # keep the goal inside the limits so the constraint interval
            # never crosses a bound (move_group rejects that with -2)
            value = float(np.clip(value, k.lo[i] + 0.012, k.hi[i] - 0.012))
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.005
            jc.tolerance_below = 0.005
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        srv = GetMotionPlan.Request()
        srv.motion_plan_request = req
        fut = self.plan_client.call_async(srv)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        if fut.result() is None:
            return None
        res = fut.result().motion_plan_response
        if res.error_code.val == 1:
            return res.trajectory.joint_trajectory
        self.get_logger().warn(f'{group} joint plan failed '
                               f'(error {res.error_code.val})')
        return None

    def plan_phase(self, targets):
        """OMPL joint-space plan for both arms; when a goal is rejected
        (e.g. the IK solution is self-colliding) the IK is re-solved with a
        fresh seed and the plan retried."""
        trajs = {}
        for group, pose in targets.items():
            for attempt in range(5):
                q = self.solve_ik(group, pose)
                if q is None:
                    self.get_logger().error(f'{group}: IK failed for target')
                    return None
                tr = self.plan_joints(group, q)
                if tr is not None:
                    trajs[group] = tr
                    break
                self.get_logger().warn(
                    f'{group}: plan rejected (attempt {attempt + 1}), '
                    f're-solving IK')
                self.last_q[group] = None
            else:
                return None
        return trajs

    def execute_synchronized(self, traj_l, traj_r, label):
        if traj_l is None or traj_r is None:
            return False
        plans = []
        for traj, joints in ((traj_l, LEFT_ARM_JOINTS),
                             (traj_r, RIGHT_ARM_JOINTS)):
            points = traj.points
            idx = [traj.joint_names.index(j) for j in joints]
            dur = points[-1].time_from_start.sec + \
                points[-1].time_from_start.nanosec * 1e-9
            plans.append((points, idx, dur))

        total = max(p[2] for p in plans) + 0.6
        ts = np.arange(0.0, total, 0.1)

        futures = []
        for (points, idx, dur), group in zip(plans, ('left_arm', 'right_arm')):
            t = np.array([p.time_from_start.sec +
                          p.time_from_start.nanosec * 1e-9 for p in points])
            mat = np.array([[p.positions[i] for i in idx] for p in points])
            jt = JointTrajectory()
            jt.joint_names = LEFT_ARM_JOINTS if group == 'left_arm' \
                else RIGHT_ARM_JOINTS
            for j in range(len(idx)):
                col = np.interp(ts, t, mat[:, j])
                for k, time_s in enumerate(ts):
                    if len(jt.points) <= k:
                        pt = JointTrajectoryPoint()
                        pt.time_from_start = ros_time(time_s)
                        pt.positions = [0.0] * len(idx)
                        jt.points.append(pt)
                    jt.points[k].positions[j] = float(col[k])
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = jt
            futures.append((group, self.traj_clients[group].send_goal_async(goal)))

        ok = True
        for group, fut in futures:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(f'{group} rejected {label}')
                ok = False
                continue
            res_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut,
                                             timeout_sec=total + 60.0)
            result = res_fut.result()
            code = result.result.error_code if result else -1
            if code != 0:
                self.get_logger().error(f'{group} failed {label} (code {code})')
                ok = False
        return ok

    # ------------------------------------------------------------------
    # planning scene: table + target object + ACM merge
    # ------------------------------------------------------------------
    def update_scene(self, name, center, size):
        """Put the table and the target object into the planning scene. The
        existing ACM is read from move_group first and merged (only the
        finger links may touch the object; letting the wrist/hand pass
        through it lets OMPL plan paths that physically smash the object)."""
        from moveit_msgs.msg import (AllowedCollisionEntry,
                                     AllowedCollisionMatrix, PlanningScene)
        from moveit_msgs.msg import PlanningSceneComponents as PSC

        hand_links = []
        for side in ('left', 'right'):
            hand_links += [f'openarm_{side}_left_finger',
                           f'openarm_{side}_right_finger']

        acm = None
        for _ in range(10):        # the scene monitor may need time on startup
            q = GetPlanningScene.Request()
            q.components.components = PSC.ALLOWED_COLLISION_MATRIX
            fut = self.scene_get_client.call_async(q)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
            if fut.result() is not None:
                acm = fut.result().scene.allowed_collision_matrix
                break
            self.get_logger().warn('get_planning_scene timed out, retrying')
        if acm is None:
            self.get_logger().error('planning scene unavailable')
            raise RuntimeError('planning scene unavailable')
        names = list(acm.entry_names)
        rows = {nm: list(ev.enabled) for nm, ev in
                zip(acm.entry_names, acm.entry_values)}

        new_names = ['work_table', name] + hand_links
        for nm in new_names:
            if nm in names:
                continue
            for row in rows.values():
                row.append(False)
            names.append(nm)
            rows[nm] = [False] * len(names)
        for nm in names:
            rows.setdefault(nm, [False] * len(names))
            while len(rows[nm]) < len(names):
                rows[nm].append(False)

        def allow(a, b):
            rows[a][names.index(b)] = True
            rows[b][names.index(a)] = True

        allow(name, 'work_table')
        for l in hand_links:
            allow(name, l)

        acm2 = AllowedCollisionMatrix()
        acm2.entry_names = names
        for nm in names:
            acm2.entry_values.append(AllowedCollisionEntry(
                enabled=[bool(v) for v in rows[nm]]))

        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.id = name
        obj.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[max(size[0], 0.03), max(size[1], 0.03),
                        max(size[2], 0.03)]))
        obj.primitive_poses.append(
            Pose(position=Point(x=float(center[0]), y=float(center[1]),
                                z=float(center[2])),
                 orientation=Quaternion(w=1.0)))
        obj.operation = CollisionObject.ADD

        table = CollisionObject()
        table.header.frame_id = 'world'
        table.id = 'work_table'
        table.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX, dimensions=[1.30, 1.00, 0.05]))
        table.primitive_poses.append(
            Pose(position=Point(x=0.0, y=-0.70, z=TABLE_TOP - 0.025),
                 orientation=Quaternion(w=1.0)))
        table.operation = CollisionObject.ADD

        req = ApplyPlanningScene.Request()
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj, table]
        scene.allowed_collision_matrix = acm2
        req.scene = scene
        fut = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    # ------------------------------------------------------------------
    # execution helpers
    # ------------------------------------------------------------------
    def execute_synchronized_trajs(self, traj_l, traj_r, label):
        return self.execute_synchronized(traj_l, traj_r, label)

    def move_grippers(self, q_left, q_right, effort=7.0):
        futures = []
        for side, q in (('left_arm', q_left), ('right_arm', q_right)):
            goal = GripperCommand.Goal()
            goal.command.position = float(q)
            goal.command.max_effort = effort
            futures.append(self.gripper_clients[side].send_goal_async(goal))
        ok = True
        for fut in futures:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            handle = fut.result()
            if handle is None:
                ok = False
                continue
            res_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut, timeout_sec=30.0)
            if res_fut.result() is None:
                ok = False
        return ok

    # ------------------------------------------------------------------
    # perception
    # ------------------------------------------------------------------
    def find_object(self, name, expected_center):
        tpl = self.templates[name]
        deadline = time.time() + 20.0
        target = np.array(tpl['size'])
        while time.time() < deadline:
            if self.use_gt:
                pose = self.gazebo_pose(name)
                if pose is not None:
                    return np.array([pose[0], pose[1], pose[2] - self.gz_z])
                return None
            dets = self.latest_detections
            if dets is not None:
                best, best_err = None, 1e9
                for det in dets.detections:
                    size = np.array([det.bbox.size.x, det.bbox.size.y,
                                     det.bbox.size.z])
                    err = np.abs(size - target).sum()
                    if err < best_err:
                        best, best_err = det, err
                if best is not None and best_err < 0.10:
                    c = best.bbox.center.position
                    return np.array([c.x, c.y, c.z])
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().warn(f'{name}: perception timeout, using start pose')
        return np.array(expected_center, float)

    def verify(self, name):
        pose = self.gazebo_pose(name)
        if pose is None:
            return False, 'entity state unavailable'
        dest = np.array(self.templates[name]['dest'])
        pos = np.array([pose[0], pose[1], pose[2] - self.gz_z])
        err = float(np.linalg.norm(pos - dest))
        return err < 0.10, f'position error {err * 100:.1f} cm'

    # ------------------------------------------------------------------
    # the full bimanual carry sequence for one object
    # ------------------------------------------------------------------
    def carry_one(self, name):
        tpl = self.templates[name]

        self.get_logger().info(f'=== {name}: reset to start pose ===')
        if not self.teleport(name, tpl['start']):
            return False

        self.get_logger().info(f'=== {name}: perceiving ===')
        center = self.find_object(name, tpl['start'])
        if center is None:
            self.get_logger().error(f'{name}: object not perceived')
            return False

        self.update_scene(name, center, tpl['size'])
        spec = self.grasp_spec(center, tpl)
        self.publish_plan_markers(name, spec)
        self.get_logger().info(f'{name}: center={np.round(center, 3).tolist()}')

        hands = spec['hands']

        def both(key):
            return {g: hands[s][key]
                    for g, s in (('left_arm', 'left'), ('right_arm', 'right'))}

        self.get_logger().info(f'=== {name}: approach ===')
        trajs = self.plan_phase(both('pre_grasp'))
        if trajs is None:
            self.get_logger().warn(f'{name}: pre-grasp unreachable, '
                                   f'approaching the grasp pose directly')
            trajs = self.plan_phase(both('grasp'))
        if trajs is None or not self.execute_synchronized(
                trajs['left_arm'], trajs['right_arm'], 'approach'):
            self.go_home_best_effort()
            return False

        self.get_logger().info(f'=== {name}: grasp ===')
        if not self.interp_phase(both('grasp'), duration=1.4):
            self.go_home_best_effort()
            return False

        self.get_logger().info(f'=== {name}: attach (left hand welds) ===')
        self.move_grippers(hands['left']['q_close'], hands['right']['q_close'])
        # Weld only the left hand: two rigid welds plus two position-
        # controlled arms over-constrain the simulation and abort the JTCs;
        # the right hand follows its own trajectory, fingertips hovering.
        self.send_grasp_cmd('attach', 'left', hands['left']['link'], name)
        if not self.wait_status('left', 'attached'):
            self.get_logger().error(f'{name}: attach failed')
            self.go_home_best_effort()
            return False

        self.get_logger().info(f'=== {name}: lift ===')
        lift = self.translated_spec(spec, np.array([0.0, 0.0, self.lift_height]))
        if not self.interp_phase({('left_arm' if s == 'left' else 'right_arm'):
                                  h['grasp']
                                  for s, h in lift['hands'].items()}):
            self.get_logger().warn(f'{name}: lift failed, carrying low')
            lift = spec

        delta = spec['dest_center'] - spec['center']
        hover = self.translated_spec(spec, delta + np.array([0.0, 0.0, 0.04]))
        self.get_logger().info(f'=== {name}: carry ===')
        # move along the straight Cartesian line in short segments so the
        # joint-space interpolation cannot dip the object into the table
        n_seg = 5
        carried = True
        for k in range(1, n_seg + 1):
            seg = self.translated_spec(spec, delta * (k / n_seg) +
                                       np.array([0.0, 0.0, self.lift_height]))
            targets = {('left_arm' if s == 'left' else 'right_arm'):
                       h['grasp'] for s, h in seg['hands'].items()}
            if not self.interp_phase(targets, duration=0.8):
                # one retry with fresh IK seeds before giving up
                if not self.interp_phase(targets, duration=0.8):
                    carried = False
                    break
        if not carried:
            self.go_home_best_effort()
            return False

        self.get_logger().info(f'=== {name}: release at destination ===')
        # release at the hover: the object drops the last ~4 cm onto the
        # table (soft contact), avoiding the least reliable IK step
        self.send_grasp_cmd('detach', 'left')
        self.wait_status('left', 'detached')

        self.get_logger().info(f'=== {name}: retreat ===')
        retreat = {}
        for side, group in (('left', 'left_arm'), ('right', 'right_arm')):
            ori = hover['hands'][side]['grasp'].orientation
            m = quat_matrix([ori.x, ori.y, ori.z, ori.w])
            back = m[:3, 2]
            retreat[group] = shifted(hover['hands'][side]['grasp'], back * 0.10)
        self.interp_phase(retreat, duration=1.2)

        # open the grippers only after retreating: opening at the object
        # would smack it away with the fingertips
        self.move_grippers(hands['left']['q_open'], hands['right']['q_open'])
        self.execute_joint_goals([0.0] * 7, [0.0] * 7, 'home', duration=3.0)
        return True

    def go_home_best_effort(self):
        """Best-effort return of both arms to the home pose."""
        self.get_logger().warn('returning arms home after failure')
        try:
            self.execute_joint_goals([0.0] * 7, [0.0] * 7, 'failure-home',
                                     duration=3.0)
            self.move_grippers(0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f'homing failed: {e}')

    def publish_plan_markers(self, name, spec):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        from std_msgs.msg import ColorRGBA
        for i, (side, h) in enumerate(spec['hands'].items()):
            for j, key in enumerate(('pre_grasp', 'grasp')):
                mk = Marker()
                mk.header.frame_id = 'world'
                mk.ns = f'{name}_{side}'
                mk.id = j
                mk.type = Marker.ARROW
                mk.action = Marker.ADD
                mk.pose = h[key]
                mk.scale.x, mk.scale.y, mk.scale.z = 0.15, 0.012, 0.012
                mk.color = (ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.9)
                            if key == 'grasp'
                            else ColorRGBA(r=0.9, g=0.9, b=0.1, a=0.6))
                markers.markers.append(mk)
        self.marker_pub.publish(markers)

    # ------------------------------------------------------------------
    def run(self):
        results = {}
        for name in self.seq:
            try:
                carried = self.carry_one(name)
            except Exception:
                import traceback
                self.get_logger().error(f'{name}: exception\n' +
                                        traceback.format_exc())
                carried = False
            ok, detail = self.verify(name)
            self.get_logger().info(
                f'>>> {name}: carry={"OK" if carried else "FAIL"} '
                f'verify={"OK" if ok else "FAIL"} ({detail})')
            results[name] = carried and ok
            self.teleport(name, self.templates[name]['park'])
        self.get_logger().info('======== BIMANUAL CARRY SUMMARY ========')
        for name, ok in results.items():
            self.get_logger().info(f'  {name:15s}: {"PASS" if ok else "FAIL"}')
        self.get_logger().info(
            f'  total: {sum(results.values())}/{len(results)} passed')


def main(args=None):
    rclpy.init(args=args)
    node = CarryNode()
    time.sleep(2.0)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
