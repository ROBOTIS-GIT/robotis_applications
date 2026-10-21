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
# Authors: Yeonguk Kim

import rclpy
from robotis_vuer.vr_publisher_sg2 import VRTrajectoryPublisher


class F2VRTrajectoryPublisher(VRTrajectoryPublisher):
    """Use SG2 VR controls with the F2 HX2-D1 gripper range."""

    def __init__(self):
        super().__init__(gripper_max_position=1.1)


def main(args=None):
    rclpy.init(args=args)
    node = F2VRTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
