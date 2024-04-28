import rclpy, time, random, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
import numpy as np


class FollowWalls(Node):
    def __init__(self):
        super().__init__('hw5')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # self.timer = self.create_timer(0.5, self.timer_callback)
        self.distances = []
        self.progress = 0
        self.initiated = 0
        self.ogdisttowall = 0
        self.walldist = 0

    def timer_callback(self):
        pass

    def scan_callback(self, msg):
        self.distances = msg.ranges
        # print('left dist ' + str(self.distances[89])) # left
        # print('front dist ' + str(self.distances[0])) # front
        # print('right dist ' + str(self.distances[269])) # right
        # print('original to current ' + str(abs(self.walldist - self.distances[89])))
        # print('diagonal diff ' + str(abs(self.distances[89-30] - self.distances[89+30])))
        # print('diagonal left ' + str(abs(self.distances[89+30])))
        # print('diagonal right ' + str(abs(self.distances[89-30])))
        if len(self.distances) > 10 and self.progress == 0:
            self.progress = 1
        if self.progress == 1:
            self.moveForward()
        if self.progress == 2:
            self.turnRight()
            

    def moveForward(self):
        msg = Twist()
        msg.linear.x = 0.5
        self.publisher.publish(msg)
        if self.distances[0] < 1:
            msg.linear.x = 0.1
            self.publisher.publish(msg)
        if abs(0.5-self.distances[0]) < 0.025: # 5% tolerance
            msg.linear.x = 0.0
            self.publisher.publish(msg)
            self.progress = 2

    def turnRight0(self):
        msg = Twist()
        msg.angular.z = -0.1
        self.publisher.publish(msg)
        time.sleep(1)
        # print('walldist ' + str(self.walldist))
        # print('left dist ' + str(self.distances[89])) # left
        print('diagonal left ' + str(abs(self.distances[89+30])))
        print('diagonal right ' + str(abs(self.distances[89-30])))
        print('diagonal diff ' + str(abs(self.distances[89-30] - self.distances[89+30])))
        if self.distances[89+30] - self.distances[89-30] < 0.03:
            self.progress = 1
            self.walldist == 0


    def turnRight1(self):
        msg = Twist()
        if self.initiated == 0:
            self.ogdisttowall = self.distances[0]
            msg.angular.z = -0.5
            self.publisher.publish(msg)
            print('initiated')
            time.sleep(1)
            self.initiated = 1
        print(abs(self.distances[89] - self.ogdisttowall))
        msg.angular.z = -0.3
        self.publisher.publish(msg)
        if abs(self.distances[89] - self.ogdisttowall) < 0.05:
            msg.angular.z = -0.05
            self.publisher.publish(msg)
            if abs(self.distances[89] - self.ogdisttowall) < 0.015:
                self.progress = 1
                self.initiated = 0

    def turnRight(self):
        msg = Twist()
        if self.initiated == 0:
            msg.angular.z = -0.5
            self.publisher.publish(msg)
            time.sleep(1)
            self.initiated = 1
        msg.angular.z = -0.3
        self.publisher.publish(msg)
        leftavg = self.getAverage(self.distances, 109, 5)
        rightavg = self.getAverage(self.distances, 69, 5)
        if abs(rightavg - leftavg) < 0.1:
            msg.angular.z = -0.02
            self.publisher.publish(msg)
            if abs(rightavg - leftavg) < 0.01 and self.distances[89] < 0.52:
                self.progress = 1
                self.initiated = 0

    def getAverage(self, list, middleentry, range):
        average = sum(list[middleentry-range:middleentry+range]) / (2*range+1)
        return average
    
    def expected_avg(max):
        sum = 0
        for theta in range(-max, max):
            sum += .5/np.acos(theta)

        return sum/max


def main(args=None):
    rclpy.init(args=args)
    insidewalls = FollowWalls()

    rclpy.spin(insidewalls)
    insidewalls.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
