#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Wonho Yun

import asyncio
import os
import socket
import threading
import traceback

from geometry_msgs.msg import Point32, Quaternion
import nest_asyncio
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from robotis_interfaces.msg import HandJoints
from std_msgs.msg import Bool
from vuer import Vuer
from vuer.schemas import Hands, HemisphereLightStage, Scene

# Allow nested asyncio execution
nest_asyncio.apply()


class VRTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('vr_trajectory_publisher')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # VR publishing control flag
        self.vr_publishing_enabled = False

        # VR Server setup
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
            queries={'grid': False, 'reconnect': True},
            queue_len=3
        )

        try:
            import vuer
            self.get_logger().info(f'Vuer version: {vuer.__version__}')
        except AttributeError:
            self.get_logger().info('Vuer version: (not set)')

        self.fps = 30
        self.get_logger().info(f'VR Trajectory server available at: https://{hostname}:8012')

        # VR event handlers
        self.vuer.add_handler('HAND_MOVE')(self.on_hand_move)

        # QoS setting
        self.vr_stream_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.left_hand_pos_pub = self.create_publisher(
            HandJoints,
            '/left_hand/hand_joint_pos',
            self.vr_stream_qos
        )
        self.right_hand_pos_pub = self.create_publisher(
            HandJoints,
            '/right_hand/hand_joint_pos',
            self.vr_stream_qos
        )

        # Reactivate topic
        self.reactivate_sub = self.create_subscription(
            Bool,
            '/reactivate',
            self.reactivate_callback,
            10
        )

        self.required_vr_frames = [0,
                                   1, 2, 3, 4,
                                   6, 7, 8, 9,
                                   11, 12, 13, 14,
                                   16, 17, 18, 19,
                                   21, 22, 23, 24]

        self.vr_hand_to_urdf = np.array([
            [0, -1, 0],
            [-1, 0, 0],
            [0, 0, -1]
        ])

        self.prev_poses_right = np.zeros((21, 3))
        self.start_poses_right = False

        self.prev_poses_left = np.zeros((21, 3))
        self.start_poses_left = False

        # Cached transform constants for hot paths
        self.vr_to_ros_matrix = np.array([
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ], dtype=np.float64)

        # VR data storage
        self.left_hand_data = None
        self.right_hand_data = None
        self.head_transform_matrix = np.eye(4)
        self.head_inverse_matrix = np.eye(4)
        self.hand_pose_is_head_relative = self.declare_parameter(
            'hand_pose_is_head_relative', True
        ).value

        # Low-pass filter settings
        self.low_pass_filter_alpha = 0.9
        self.pose_filters = {}

        self.hand_log_counter = 0
        self.wrist_debug_log_counter = 0
        self.wrist_debug_log_every_n = 30

        self.status_timer = self.create_timer(3.0, self.log_status)

        # Async event loop for Vuer server
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.start_vuer_server()

        self.get_logger().info('VR Trajectory Publisher node has been started')
        vr_status = 'ENABLED' if self.vr_publishing_enabled else 'DISABLED'
        self.get_logger().info(
            f'VR publishing is {vr_status} by default. '
            'Publish std_msgs/Bool on /reactivate to set enable/disable.'
        )

    def reactivate_callback(self, msg):
        """Set VR publishing from /reactivate Bool message."""
        new_state = bool(msg.data)
        self._set_vr_publishing_enabled(new_state)

    def _set_vr_publishing_enabled(self, new_state):
        """Apply VR publishing on/off and the same side effects as the old Bool topic."""
        new_state = bool(new_state)
        self.vr_publishing_enabled = new_state
        # status = 'ENABLED' if self.vr_publishing_enabled else 'DISABLED'
        # self.get_logger().info(f'VR publishing set to: {status}')

        if self.vr_publishing_enabled:
            self.start_poses_left = False
            self.start_poses_right = False
            self.prev_poses_left.fill(0.0)
            self.prev_poses_right.fill(0.0)
        else:
            self.start_poses_left = False
            self.start_poses_right = False
            self.prev_poses_left.fill(0.0)
            self.prev_poses_right.fill(0.0)
            self.pose_filters.clear()

    def log_status(self):
        """Log current system status for debugging."""
        vr_status = 'ENABLED' if self.vr_publishing_enabled else 'DISABLED'
        self.get_logger().info(f'Status: VR={vr_status}')

    def is_valid_float(self, value):
        """Check if value is valid float (excluding NaN, inf)."""
        return isinstance(value, (int, float)) and np.isfinite(value)

    # Body head-relative frame from head_inverse @ world:
    # +Y=forward, +Z=right, +X=down. Convert to ROS (+X forward, +Y left, +Z up).
    BODY_HEAD_TO_ROS_POSITION = np.array([
        [0, 1, 0],    # ROS X = head +Y (forward)
        [0, 0, -1],   # ROS Y = -head Z (left)
        [-1, 0, 0]    # ROS Z = -head X (up)
    ])

    def quat_multiply(self, q1, q2):
        """Multiply two quaternions."""
        w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        msg = Quaternion()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.w = w
        return msg

    def process_hand_joints(self, hand_data, side='left'):
        """Process VR hand data (retarget) and publish HandJoints."""
        if hand_data is None or len(hand_data) != 400:
            return

        hand_joints = HandJoints()
        hand_joints.header.stamp = self.get_clock().now().to_msg()
        hand_joints.header.frame_id = ''
        hand_joints.joints = []
        temp_joints = np.zeros((21, 3), dtype=np.float64)
        pose_counter = 0
        wrist_rot = np.eye(3, dtype=np.float64)

        hand_arr = (
            hand_data if isinstance(hand_data, np.ndarray)
            else np.asarray(hand_data, dtype=np.float64)
        )
        head_rot_inv = (
            self.head_inverse_matrix[:3, :3]
            if self.hand_pose_is_head_relative else None
        )
        head_world_pos = (
            self.head_transform_matrix[:3, 3]
            if self.hand_pose_is_head_relative else None
        )

        try:
            for i in self.required_vr_frames:
                start = i * 16
                world_joint_matrix = hand_arr[start:start + 16].reshape(4, 4, order='F')
                world_rot = world_joint_matrix[:3, :3]
                world_pos = world_joint_matrix[:3, 3]

                if (self.hand_pose_is_head_relative and head_rot_inv is not None
                        and head_world_pos is not None):
                    relative_pos_vr = head_rot_inv @ (world_pos - head_world_pos)
                    temp_joints[pose_counter, :] = relative_pos_vr
                    if i == 0:
                        wrist_rot = head_rot_inv @ world_rot
                else:
                    relative_pos_vr = world_pos
                    temp_joints[pose_counter, :] = relative_pos_vr

                pose_counter += 1
        except Exception as e:
            self.get_logger().warn(f'Error processing hand joints for {side}: {e}')
            return

        if pose_counter != 21:
            return

        if side == 'left':
            if self.start_poses_left:
                temp_joints = (
                    self.low_pass_filter_alpha * temp_joints
                    + (1.0 - self.low_pass_filter_alpha) * self.prev_poses_left
                )
            self.prev_poses_left[:] = temp_joints
            self.start_poses_left = True
            hand_publisher = self.left_hand_pos_pub
        elif side == 'right':
            if self.start_poses_right:
                temp_joints = (
                    self.low_pass_filter_alpha * temp_joints
                    + (1.0 - self.low_pass_filter_alpha) * self.prev_poses_right
                )
            self.prev_poses_right[:] = temp_joints
            self.start_poses_right = True
            hand_publisher = self.right_hand_pos_pub
        else:
            return

        rel_points = temp_joints - temp_joints[0]
        retarget_points = (self.vr_hand_to_urdf @ (wrist_rot.T @ rel_points.T)).T
        for p in retarget_points:
            msg_p = Point32()
            msg_p.x = float(p[0])
            msg_p.y = float(p[1])
            msg_p.z = float(p[2])
            hand_joints.joints.append(msg_p)

        hand_publisher.publish(hand_joints)

    def start_vuer_server(self):
        """Start the VR server in a separate thread."""
        def run_server():
            try:
                # Set event loop for this thread
                asyncio.set_event_loop(self.loop)
                self.get_logger().info('Starting VR server...')
                # Register handler and start server
                # spawn(start=True) calls Vuer.start() -> Server.start()
                # Server.start() internally calls run_until_complete + run_forever (blocks here)
                self.vuer.spawn(start=True)(self.main_hand_tracking)
            except Exception as e:
                self.get_logger().error(f'Error in VR server thread: {e}')
                self.get_logger().error(traceback.format_exc())

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

    async def main_hand_tracking(self, session):
        """Run main hand tracking session."""
        try:
            fps = self.fps
            self.current_session = session
            self.get_logger().info('Starting hand tracking session')

            bg_children = [
                HemisphereLightStage(key='light-stage', hide=False),
                Hands(
                    fps=fps,
                    stream=True,
                    key='hands',
                    hideLeft=False,
                    hideRight=False,
                ),
            ]

            session.set @ Scene(
                bgChildren=bg_children,
            )
            while True:
                await asyncio.sleep(1/fps)
        except Exception as e:
            self.get_logger().error(f'Error in hand tracking session: {e}')

    async def on_hand_move(self, event, session):
        """Handle hand movement events."""
        try:
            if not self.vr_publishing_enabled:
                return
            if not isinstance(event.value, dict):
                return

            if 'left' in event.value:
                left_data = event.value['left']
                if isinstance(left_data, (list, np.ndarray)) and len(left_data) == 400:
                    self.left_hand_data = (
                        left_data if isinstance(left_data, np.ndarray)
                        else np.asarray(left_data, dtype=np.float64)
                    )
                if self.left_hand_data is not None:
                    self.process_hand_joints(self.left_hand_data, 'left')
            if 'right' in event.value:
                right_data = event.value['right']
                if isinstance(right_data, (list, np.ndarray)) and len(right_data) == 400:
                    self.right_hand_data = (
                        right_data if isinstance(right_data, np.ndarray)
                        else np.asarray(right_data, dtype=np.float64)
                    )
                if self.right_hand_data is not None:
                    self.process_hand_joints(self.right_hand_data, 'right')

        except Exception as e:
            self.get_logger().error(f'Error in hand move event: {e}')

    def __del__(self):
        try:
            if hasattr(self, 'vuer') and hasattr(self.vuer, 'stop'):
                self.vuer.stop()
        except Exception as e:
            self.get_logger().error(f'Error in cleanup: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VRTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
