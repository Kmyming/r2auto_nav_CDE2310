# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np


class Scanner(Node):

    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # convert ranges to numpy and ignore zero (invalid) readings
        laser_range = np.array(msg.ranges, dtype=float)
        laser_range[laser_range == 0.0] = np.nan

        if np.all(np.isnan(laser_range)):
            self.get_logger().warn('Laser scan contains no valid range data')
            return

        closest_index = int(np.nanargmin(laser_range))
        closest_distance = float(laser_range[closest_index])

        # compute the angle in degrees using LaserScan metadata instead of a hard-coded scale
        angle_rad = msg.angle_min + closest_index * msg.angle_increment
        angle_deg = float(np.rad2deg(angle_rad))

        self.get_logger().info(
            f'Closest object {closest_distance:.3f} m at {angle_deg:.1f} deg (index {closest_index})'
        )


def main(args=None):
    rclpy.init(args=args)
    scanner = Scanner()
    rclpy.spin(scanner)
    scanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
