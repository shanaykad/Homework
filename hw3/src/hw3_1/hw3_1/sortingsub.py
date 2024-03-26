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

import rclpy

from std_msgs.msg import UInt32MultiArray

g_node = None

combined_list = []

global visited1, visited2
visited1 = False
visited2 = False

def chatter_callback1(msg):
    global visited1, list1, g_node
    visited1 = True
    list1 = list(msg.data)

def chatter_callback2(msg):
    global visited2, list2, g_node
    visited2 = True
    list2 = list(msg.data)

def timer_callback():
    global visited1, visited2, list1, list2
    # print("visisted1 is", visited1)
    # print("visisted2 is", visited2)
    if(visited1 and visited2):

        index = list1.pop(0)
        list2.pop(0)
        list3 =sorted(list1+list2)

        g_node.get_logger().info('The serial number is: "%s"' % index)
        g_node.get_logger().info('The sorted list is: "%s"' % list3)
        
        # print("The serial number is: ", index)
        # print("The sorted list is: ", list3)

        visited1 = False
        visited2 = False

        publisher = g_node.create_publisher(UInt32MultiArray, 'topic_3', 10)
        msg3 = UInt32MultiArray()
        msg3.data = [index] + list3
        publisher.publish(msg3)
        g_node.get_logger().info('Publishing topic 3: "%s"' % msg3.data)

def main(args=None):
    global g_node
    # global list1, list2
    rclpy.init(args=args)
    visited1 = False
    visited2 = False
    g_node = rclpy.create_node('sortingsubnode')

    subscription1 = g_node.create_subscription(UInt32MultiArray, 'topic_1', chatter_callback1, 10)
    subscription2 = g_node.create_subscription(UInt32MultiArray, 'topic_2', chatter_callback2, 10)
    subscription1  # prevent unused variable warning
    subscription2
    
    timer = g_node.create_timer(2, timer_callback)

    while rclpy.ok():
        rclpy.spin(g_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()