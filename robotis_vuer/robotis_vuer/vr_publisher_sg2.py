#!/usr/bin/env python3

import asyncio
import os
import socket
import threading
import traceback

from geometry_msgs.msg import PoseStamped
import nest_asyncio
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from vuer import Vuer
from vuer.schemas import Body, MotionControllers


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

BODY_JOINT_INDEX = {name: index for index, name in enumerate(BODY_JOINT_KEYS)}


class VRRawControllerPublisher(Node):
    # WebXR/Vuer world axes: +X right, +Y up, -Z forward.
    # ROS world axes: +X forward, +Y left, +Z up.
    VR_WORLD_TO_ROS_WORLD_MATRIX = np.array([
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ], dtype=np.float64)

    def __init__(self):
        super().__init__('vr_raw_controller_publisher')

        self.vr_publishing_enabled = True
        self.vr_world_to_ros_world_rot = R.from_matrix(self.VR_WORLD_TO_ROS_WORLD_MATRIX)
        self.vr_stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.head_pose_ros_pub = self.create_publisher(PoseStamped, '/vr/raw/controller/head_pose_ros', self.vr_stream_qos)
        self.left_elbow_ros_pub = self.create_publisher(PoseStamped, '/vr/raw/controller/left_elbow_pose_ros', self.vr_stream_qos)
        self.right_elbow_ros_pub = self.create_publisher(PoseStamped, '/vr/raw/controller/right_elbow_pose_ros', self.vr_stream_qos)
        self.left_controller_pose_ros_pub = self.create_publisher(PoseStamped, '/vr/raw/controller/left_pose_ros', self.vr_stream_qos)
        self.right_controller_pose_ros_pub = self.create_publisher(PoseStamped, '/vr/raw/controller/right_pose_ros', self.vr_stream_qos)
        self.left_controller_state_pub = self.create_publisher(Joy, '/vr/raw/controller/left_state', self.vr_stream_qos)
        self.right_controller_state_pub = self.create_publisher(Joy, '/vr/raw/controller/right_state', self.vr_stream_qos)

        self.create_subscription(Bool, '/vr_control/toggle', self.vr_control_callback, 10)

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
        self.vuer.add_handler('BODY_MOVE')(self.on_body_tracking_move)
        self.vuer.add_handler('CONTROLLER_MOVE')(self.on_controller_move)

        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.start_vuer_server()

        self.get_logger().info('VR raw controller publisher started')

    def vr_control_callback(self, msg):
        self.vr_publishing_enabled = bool(msg.data)
        status = 'enabled' if self.vr_publishing_enabled else 'disabled'
        self.get_logger().info(f'VR raw controller publishing {status}')

    def start_vuer_server(self):
        def run_server():
            try:
                asyncio.set_event_loop(self.loop)
                self.vuer.spawn(start=True)(self.main_controller_tracking)
            except Exception as exc:
                self.get_logger().error(f'Error in VR raw controller thread: {exc}')
                self.get_logger().error(traceback.format_exc())

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

    async def main_controller_tracking(self, session):
        fps = 30
        session.upsert(
            MotionControllers(stream=True, key='motion-controller', left=True, right=True),
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

        stamp = self.get_clock().now().to_msg()

        self.publish_ros_pose_from_matrix(self.head_pose_ros_pub, stamp, head_matrix)

        left_elbow = self.get_body_joint_matrix_from_flat(body_array, 'left-arm-lower')
        if left_elbow is not None:
            self.publish_ros_pose_from_matrix(self.left_elbow_ros_pub, stamp, left_elbow)

        right_elbow = self.get_body_joint_matrix_from_flat(body_array, 'right-arm-lower')
        if right_elbow is not None:
            self.publish_ros_pose_from_matrix(self.right_elbow_ros_pub, stamp, right_elbow)

    async def on_controller_move(self, event, _session):
        if not self.vr_publishing_enabled or not rclpy.ok():
            return
        if not isinstance(event.value, dict):
            return

        stamp = self.get_clock().now().to_msg()
        left_state = event.value.get('leftState')
        if isinstance(left_state, dict):
            self.left_controller_state_pub.publish(
                self.make_controller_state_msg(stamp, left_state)
            )

        right_state = event.value.get('rightState')
        if isinstance(right_state, dict):
            self.right_controller_state_pub.publish(
                self.make_controller_state_msg(stamp, right_state)
            )

        left_matrix_raw = event.value.get('left')
        if isinstance(left_matrix_raw, (list, np.ndarray)) and len(left_matrix_raw) == 16:
            left_matrix = np.asarray(left_matrix_raw, dtype=np.float64).reshape(4, 4, order='F')
            self.publish_ros_pose_from_matrix(self.left_controller_pose_ros_pub, stamp, left_matrix)

        right_matrix_raw = event.value.get('right')
        if isinstance(right_matrix_raw, (list, np.ndarray)) and len(right_matrix_raw) == 16:
            right_matrix = np.asarray(right_matrix_raw, dtype=np.float64).reshape(4, 4, order='F')
            self.publish_ros_pose_from_matrix(self.right_controller_pose_ros_pub, stamp, right_matrix)

    def make_controller_state_msg(self, stamp, state):
        msg = Joy()
        msg.header.stamp = stamp
        msg.header.frame_id = ''
        msg.axes = [
            float(state.get('thumbstickValue', [0.0, 0.0])[0]) if len(state.get('thumbstickValue', [0.0, 0.0])) > 0 else 0.0,
            float(state.get('thumbstickValue', [0.0, 0.0])[1]) if len(state.get('thumbstickValue', [0.0, 0.0])) > 1 else 0.0,
            float(state.get('triggerValue', 0.0)),
            float(state.get('squeezeValue', 0.0)),
        ]
        msg.buttons = [
            1 if bool(state.get('thumbstick', False)) else 0,
            1 if bool(state.get('aButton', False)) else 0,
            1 if bool(state.get('bButton', False)) else 0,
            1 if bool(state.get('xButton', False)) else 0,
            1 if bool(state.get('yButton', False)) else 0,
        ]
        return msg

    def get_body_joint_matrix_from_flat(self, body_array, joint_name):
        index = BODY_JOINT_INDEX.get(joint_name)
        if index is None:
            return None
        start = index * 16
        end = start + 16
        if body_array.size < end:
            return None
        matrix = body_array[start:end].reshape(4, 4, order='F')
        if abs(float(np.linalg.det(matrix[:3, :3]))) < 1e-6:
            return None
        return matrix

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

    def publish_ros_pose_from_matrix(self, publisher, stamp, matrix):
        position, quaternion = self.matrix_to_pose(matrix)
        position_ros, quaternion_ros = self.vr_world_to_ros_transform(position, quaternion)
        publisher.publish(self.make_pose_stamped(stamp, 'ros_world', position_ros, quaternion_ros))

    def matrix_to_pose(self, mat):
        pos = mat[:3, 3]
        quat = R.from_matrix(mat[:3, :3]).as_quat()
        return pos, quat

    def vr_world_to_ros_transform(self, vr_pos, vr_quat):
        ros_pos = self.VR_WORLD_TO_ROS_WORLD_MATRIX @ np.asarray(vr_pos, dtype=np.float64)
        vr_rotation = R.from_quat(vr_quat)
        ros_rotation = self.vr_world_to_ros_world_rot * vr_rotation * self.vr_world_to_ros_world_rot.inv()
        return ros_pos, ros_rotation.as_quat()


def main(args=None):
    rclpy.init(args=args)
    node = VRRawControllerPublisher()
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
