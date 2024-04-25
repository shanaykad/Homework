import rclpy, time, random, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist


class FollowWalls(Node):
    FORWARD = 0
    LEFT = 1
    RIGHT = 2

    def __init__(self):
        super().__init__('hw5')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.distances = []
        self.state = self.FORWARD

    def timer_callback(self):
        if self.state == self.FORWARD:
            msg = Twist()
            msg.linear.x = 0.3
            msg.angular.z = 0.0
            self.publisher.publish(msg)


    def scan_callback(self, msg):
        # print(msg.ranges[360])
        msg2 = Twist()

        if self.state == self.FORWARD and self.min_subarray(msg.ranges, 360, 25) < .5:
            if self.min_subarray(msg.ranges, 180, 20) < .3:
                self.state = self.RIGHT
            else:
                self.state = self.LEFT

        elif self.state == self.RIGHT:
            msg2.linear.x = 0.0
            msg2.angular.z = 1.0
            self.publisher.publish(msg2)

        elif self.state == self.LEFT:
            msg2.linear.x = 0.0
            msg2.angular.z = -1.0
            self.publisher.publish(msg2)

        if (self.state == self.LEFT or self.state == self.RIGHT) and self.min_subarray(msg.ranges, 360, 20) > .6:
            msg2.linear.x = 0.0
            msg2.angular.z = 0.0
            self.publisher.publish(msg2)
            self.state = self.FORWARD

        time.sleep(.1)
            

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
