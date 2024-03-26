#!/usr/bin/env python3

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

from time import sleep

import rclpy
import random

from std_msgs.msg import UInt32MultiArray

# We do not recommend this style as ROS 2 provides timers for this purpose,
# and it is recommended that all nodes call a variation of spin.
# This example is only included for completeness because it is similar to examples in ROS 1.
# For periodic publication please see the other examples using timers.


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('sortingpubnode')

    publisher1 = node.create_publisher(UInt32MultiArray, 'topic_1', 10)
    publisher2 = node.create_publisher(UInt32MultiArray, 'topic_2', 10)

    msg1 = UInt32MultiArray()
    msg2 = UInt32MultiArray()

    i = 1
    while rclpy.ok():
        list1 = []
        list2 = []
        for j in range (10):
            list1.append(random.randint(0,100))
            list2.append(random.randint(0,100))

        msg1.data = [i] + list1
        msg2.data = [i] + list2
        i += 1
        node.get_logger().info('Publishing list 1: "%s"' % msg1.data)
        node.get_logger().info('Publishing list 2: "%s"' % msg2.data)
        publisher1.publish(msg1)
        publisher2.publish(msg2)
        sleep(2)  # seconds

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()