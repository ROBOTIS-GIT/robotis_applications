#!/usr/bin/env python3

import asyncio
import os
import socket
import threading
import traceback

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import nest_asyncio
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from vuer import Vuer
from vuer.schemas import Body, Hands


nest_asyncio.apply()


BODY_JOINT_KEYS = [
    "hips",
    "spine-lower",
    "spine-middle",
    "spine-upper",
    "chest",
    "neck",
    "head",
    "left-shoulder",
    "left-scapula",
    "left-arm-upper",
    "left-arm-lower",
    "left-hand-wrist-twist",
    "right-shoulder",
    "right-scapula",
    "right-arm-upper",
    "right-arm-lower",
    "right-hand-wrist-twist",
    "left-hand-palm",
    "left-hand-wrist",
    "left-hand-thumb-metacarpal",
    "left-hand-thumb-phalanx-proximal",
    "left-hand-thumb-phalanx-distal",
    "left-hand-thumb-tip",
    "left-hand-index-metacarpal",
    "left-hand-index-phalanx-proximal",
    "left-hand-index-phalanx-intermediate",
    "left-hand-index-phalanx-distal",
    "left-hand-index-tip",
    "left-hand-middle-phalanx-metacarpal",
    "left-hand-middle-phalanx-proximal",
    "left-hand-middle-phalanx-intermediate",
    "left-hand-middle-phalanx-distal",
    "left-hand-middle-tip",
    "left-hand-ring-metacarpal",
    "left-hand-ring-phalanx-proximal",
    "left-hand-ring-phalanx-intermediate",
    "left-hand-ring-phalanx-distal",
    "left-hand-ring-tip",
    "left-hand-little-metacarpal",
    "left-hand-little-phalanx-proximal",
    "left-hand-little-phalanx-intermediate",
    "left-hand-little-phalanx-distal",
    "left-hand-little-tip",
    "right-hand-palm",
    "right-hand-wrist",
    "right-hand-thumb-metacarpal",
    "right-hand-thumb-phalanx-proximal",
    "right-hand-thumb-phalanx-distal",
    "right-hand-thumb-tip",
    "right-hand-index-metacarpal",
    "right-hand-index-phalanx-proximal",
    "right-hand-index-phalanx-intermediate",
    "right-hand-index-phalanx-distal",
    "right-hand-index-tip",
    "right-hand-middle-metacarpal",
    "right-hand-middle-phalanx-proximal",
    "right-hand-middle-phalanx-intermediate",
    "right-hand-middle-phalanx-distal",
    "right-hand-middle-tip",
    "right-hand-ring-metacarpal",
    "right-hand-ring-phalanx-proximal",
    "right-hand-ring-phalanx-intermediate",
    "right-hand-ring-phalanx-distal",
    "right-hand-ring-tip",
    "right-hand-little-metacarpal",
    "right-hand-little-phalanx-proximal",
    "right-hand-little-phalanx-intermediate",
    "right-hand-little-phalanx-distal",
    "right-hand-little-tip",
    "left-upper-leg",
    "left-lower-leg",
    "left-foot-ankle-twist",
    "left-foot-ankle",
    "left-foot-subtalar",
    "left-foot-transverse",
    "left-foot-ball",
    "right-upper-leg",
    "right-lower-leg",
    "right-foot-ankle-twist",
    "right-foot-ankle",
    "right-foot-subtalar",
    "right-foot-transverse",
    "right-foot-ball",
]


class VRRawStatePublisher(Node):
    BODY_HEAD_TO_ROS_POSITION = np.array([
        [0.0, 1.0, 0.0],
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
    ], dtype=np.float64)

    VR_TO_ROS_MATRIX = np.array([
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ], dtype=np.float64)

    REQUIRED_VR_FRAMES = [
        0,
        1, 2, 3, 4,
        6, 7, 8, 9,
        11, 12, 13, 14,
        16, 17, 18, 19,
        21, 22, 23, 24,
    ]

    def __init__(self):
        super().__init__('vr_raw_state_publisher')

        self.declare_parameter('hand_pose_is_head_relative', True)
        self.hand_pose_is_head_relative = bool(
            self.get_parameter('hand_pose_is_head_relative').value
        )
        self.vr_publishing_enabled = True

        self.left_hand_data = None
        self.right_hand_data = None
        self.head_transform_matrix = np.eye(4, dtype=np.float64)
        self.head_inverse_matrix = np.eye(4, dtype=np.float64)
        self.body_head_to_ros_rot = R.from_matrix(self.BODY_HEAD_TO_ROS_POSITION)

        self.head_pub = self.create_publisher(PoseStamped, '/vr/raw/head_pose', 10)
        self.left_wrist_pub = self.create_publisher(PoseStamped, '/vr/raw/left_wrist_pose', 10)
        self.right_wrist_pub = self.create_publisher(PoseStamped, '/vr/raw/right_wrist_pose', 10)
        self.left_elbow_pub = self.create_publisher(PoseStamped, '/vr/raw/left_elbow_pose', 10)
        self.right_elbow_pub = self.create_publisher(PoseStamped, '/vr/raw/right_elbow_pose', 10)
        self.left_hand_pub = self.create_publisher(PoseArray, '/vr/raw/left_hand_points', 10)
        self.right_hand_pub = self.create_publisher(PoseArray, '/vr/raw/right_hand_points', 10)

        self.vr_control_sub = self.create_subscription(
            Bool,
            '/vr_control/toggle',
            self.vr_control_callback,
            10,
        )

        current_dir = os.path.dirname(os.path.abspath(__file__))
        cert_file = os.path.join(current_dir, 'cert.pem')
        key_file = os.path.join(current_dir, 'key.pem')
        hostname = socket.gethostbyname(socket.gethostname())
        ws_url = f'ws://{hostname}:8012'

        self.vuer = Vuer(
            host='0.0.0.0',
            port=8012,
            cert=cert_file,
            key=key_file,
            ws=ws_url,
            queries=dict(grid=False, reconnect=True),
            queue_len=3,
        )
        self.vuer.add_handler('HAND_MOVE')(self.on_hand_move)
        self.vuer.add_handler('BODY_MOVE')(self.on_body_tracking_move)

        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.start_vuer_server()

        self.get_logger().info('VR raw state publisher started')
        self.get_logger().info(
            'Publishers: /vr/raw/head_pose, /vr/raw/*_wrist_pose, /vr/raw/*_elbow_pose, /vr/raw/*_hand_points'
        )

    def vr_control_callback(self, msg):
        self.vr_publishing_enabled = bool(msg.data)
        if self.vr_publishing_enabled:
            self.left_hand_data = None
            self.right_hand_data = None
            self.get_logger().info('VR raw publishing enabled')
        else:
            self.get_logger().info('VR raw publishing disabled')

    def start_vuer_server(self):
        def run_server():
            try:
                asyncio.set_event_loop(self.loop)
                self.vuer.spawn(start=True)(self.main_hand_tracking)
            except Exception as exc:
                self.get_logger().error(f'Error in VR raw server thread: {exc}')
                self.get_logger().error(traceback.format_exc())

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

    async def main_hand_tracking(self, session):
        fps = 30
        session.upsert(
            [Hands(fps=fps, stream=True, key='hands', hideLeft=False, hideRight=False)],
            to='bgChildren',
        )
        session.upsert(
            Body(
                key='body_tracking',
                stream=True,
                fps=fps,
                leftHand=False,
                rightHand=False,
                hideIndicate=False,
                showFrame=True,
                showBody=True,
                frameScale=0.02,
            ),
            to='children',
        )
        while True:
            await asyncio.sleep(1.0 / fps)

    async def on_hand_move(self, event, _session):
        if not self.vr_publishing_enabled:
            return
        if not isinstance(event.value, dict):
            return

        left_data = event.value.get('left')
        right_data = event.value.get('right')

        if isinstance(left_data, (list, np.ndarray)) and len(left_data) == 400:
            self.left_hand_data = np.asarray(left_data, dtype=np.float64)
        if isinstance(right_data, (list, np.ndarray)) and len(right_data) == 400:
            self.right_hand_data = np.asarray(right_data, dtype=np.float64)

    async def on_body_tracking_move(self, event, _session):
        if not self.vr_publishing_enabled or not rclpy.ok():
            return
        if not isinstance(event.value, dict):
            return

        body_data = event.value.get('body')
        if not isinstance(body_data, (list, tuple, np.ndarray)):
            return

        body_array = np.asarray(body_data, dtype=np.float64)
        head_matrix = self.get_body_joint_matrix_from_flat(body_array, 'head')
        if head_matrix is None:
            return

        self.head_transform_matrix = head_matrix
        self.head_inverse_matrix = np.linalg.inv(head_matrix)
        stamp = self.get_clock().now().to_msg()

        head_pos, head_quat = self.matrix_to_pose(head_matrix)
        head_pos_ros, head_quat_ros = self.vr_to_ros_transform(head_pos, head_quat)
        self.head_pub.publish(
            self.make_pose_stamped(stamp, 'vr_world', head_pos_ros, head_quat_ros)
        )

        left_elbow = self.get_body_joint_matrix_from_flat(body_array, 'left-arm-lower')
        if left_elbow is not None:
            self.left_elbow_pub.publish(
                self.make_relative_body_pose(stamp, left_elbow)
            )

        right_elbow = self.get_body_joint_matrix_from_flat(body_array, 'right-arm-lower')
        if right_elbow is not None:
            self.right_elbow_pub.publish(
                self.make_relative_body_pose(stamp, right_elbow)
            )

        if self.left_hand_data is not None:
            wrist_msg, hand_msg = self.make_hand_messages(stamp, self.left_hand_data)
            if wrist_msg is not None and hand_msg is not None:
                self.left_wrist_pub.publish(wrist_msg)
                self.left_hand_pub.publish(hand_msg)

        if self.right_hand_data is not None:
            wrist_msg, hand_msg = self.make_hand_messages(stamp, self.right_hand_data)
            if wrist_msg is not None and hand_msg is not None:
                self.right_wrist_pub.publish(wrist_msg)
                self.right_hand_pub.publish(hand_msg)

    def make_relative_body_pose(self, stamp, joint_matrix):
        relative_joint_matrix = self.head_inverse_matrix @ joint_matrix
        pos_head, quat_head = self.matrix_to_pose(relative_joint_matrix)
        pos_ros = (self.BODY_HEAD_TO_ROS_POSITION @ pos_head).astype(np.float64)
        rel_rot = R.from_matrix(relative_joint_matrix[:3, :3])
        quat_ros = (self.body_head_to_ros_rot * rel_rot).as_quat()
        return self.make_pose_stamped(stamp, 'vr_head', pos_ros, quat_ros)

    def make_hand_messages(self, stamp, hand_data):
        poses = []
        wrist_pose = None
        arr = np.asarray(hand_data, dtype=np.float64)

        head_rot_inv = self.head_inverse_matrix[:3, :3] if self.hand_pose_is_head_relative else None
        head_world_pos = self.head_transform_matrix[:3, 3] if self.hand_pose_is_head_relative else None

        for frame_index in self.REQUIRED_VR_FRAMES:
            start = frame_index * 16
            joint_matrix = arr[start:start + 16].reshape(4, 4, order='F')
            world_rot = joint_matrix[:3, :3]
            world_pos = joint_matrix[:3, 3]

            if self.hand_pose_is_head_relative and head_rot_inv is not None and head_world_pos is not None:
                relative_pos_vr = head_rot_inv @ (world_pos - head_world_pos)
                relative_rot = head_rot_inv @ world_rot
                pos_ros = (self.BODY_HEAD_TO_ROS_POSITION @ relative_pos_vr).astype(np.float64)
                quat_ros = (self.body_head_to_ros_rot * R.from_matrix(relative_rot)).as_quat()
                frame_id = 'vr_head'
            else:
                pos_ros = self.VR_TO_ROS_MATRIX @ world_pos
                quat_ros = R.from_matrix(self.VR_TO_ROS_MATRIX @ world_rot).as_quat()
                frame_id = 'vr_world'

            pose = Pose()
            pose.position.x = float(pos_ros[0])
            pose.position.y = float(pos_ros[1])
            pose.position.z = float(pos_ros[2])
            pose.orientation.x = float(quat_ros[0])
            pose.orientation.y = float(quat_ros[1])
            pose.orientation.z = float(quat_ros[2])
            pose.orientation.w = float(quat_ros[3])
            poses.append(pose)

            if frame_index == 0:
                wrist_pose = self.make_pose_stamped(stamp, frame_id, pos_ros, quat_ros)

        if wrist_pose is None or len(poses) != len(self.REQUIRED_VR_FRAMES):
            return None, None

        hand_msg = PoseArray()
        hand_msg.header.stamp = stamp
        hand_msg.header.frame_id = wrist_pose.header.frame_id
        hand_msg.poses = poses
        return wrist_pose, hand_msg

    def make_pose_stamped(self, stamp, frame_id, position, quaternion):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = float(quaternion[0])
        msg.pose.orientation.y = float(quaternion[1])
        msg.pose.orientation.z = float(quaternion[2])
        msg.pose.orientation.w = float(quaternion[3])
        return msg

    def get_body_joint_matrix_from_flat(self, body_array, joint_name):
        if joint_name not in BODY_JOINT_KEYS:
            return None
        index = BODY_JOINT_KEYS.index(joint_name)
        start = index * 16
        end = start + 16
        if body_array.size < end:
            return None
        mat4 = body_array[start:end].reshape(4, 4, order='F')
        if abs(np.linalg.det(mat4[:3, :3])) < 1e-6:
            return None
        return mat4

    def matrix_to_pose(self, mat):
        pos = mat[:3, 3]
        quat = R.from_matrix(mat[:3, :3]).as_quat()
        return pos, quat

    def vr_to_ros_transform(self, vr_pos, vr_quat):
        ros_pos = self.VR_TO_ROS_MATRIX @ vr_pos
        vr_rot = R.from_quat(vr_quat).as_matrix()
        ros_quat = R.from_matrix(self.VR_TO_ROS_MATRIX @ vr_rot).as_quat()
        return ros_pos, ros_quat


def main(args=None):
    rclpy.init(args=args)
    node = VRRawStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
