import rclpy, time, random, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist


class FollowWalls(Node):
    def __init__(self):
        super().__init__('hw5')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # self.timer = self.create_timer(0.5, self.timer_callback)
        self.distances = []

    def timer_callback(self):
        pass


    def scan_callback(self, msg):
        self.distances = msg.ranges
        print(self.distances[0]) #print out front dist
        if len(self.distances) > 10:
            self.progress = 0
        if self.progress == 0:
            self.initializebot()
            self.progress = 1 

    def initializebot(self):
        msg = Twist()
        msg.linear.x = 0.5
        self.publisher.publish(msg)
        if self.distances[0] < 0.5:
            msg.linear.x = -0.1
            self.publisher.publish(msg)
        if abs(0.5-self.distances[0]) < 0.05: # 10% tolerance
            msg.linear.x = 0.0
            self.publisher.publish(msg)
            

    def min_subarray(self, array, mid, range):
        return min(array[mid - range:mid + range + 1]) 


def main(args=None):
    rclpy.init(args=args)
    insidewalls = FollowWalls()

    rclpy.spin(insidewalls)
    insidewalls.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
