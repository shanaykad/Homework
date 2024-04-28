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
        self.progress = 0
        self.initiated = 0
        self.ogdisttowall = 0
        self.walldist = 0

    def timer_callback(self):
        pass

    def scan_callback(self, msg):
        self.distances = msg.ranges
        if len(self.distances) > 10 and self.progress == 0:
            # print('left dist ' + str(self.distances[89])) # left
            # print('front dist ' + str(self.distances[0])) # front
            # print('right dist ' + str(self.distances[269])) # right
            # print('original to current ' + str(abs(self.walldist - self.distances[89])))
            # print('diagonal diff ' + str(abs(self.distances[89-30] - self.distances[89+30])))
            # print('diagonal left ' + str(abs(self.distances[89+30])))
            # print('diagonal right ' + str(abs(self.distances[89-30])))
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

    def turnRight(self):
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


    def turnRight2(self):
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

    def turnRight3(self):
        msg = Twist()
        if self.initiated == 0:
            msg.angular.z = -0.5
            self.publisher.publish(msg)
            time.sleep(1)
            self.initiated = 1
        msg.angular.z = -0.3
        self.publisher.publish(msg)
        if abs(self.distances[89-20] - self.distances[89+20]) < 0.1:
            msg.angular.z = -0.05
            self.publisher.publish(msg)
            if abs(self.distances[89-20] - self.distances[89+20]) < 0.015 and self.distances[89] < 0.55:
                self.progress = 1
                self.initiated = 0


def main(args=None):
    rclpy.init(args=args)
    insidewalls = FollowWalls()

    rclpy.spin(insidewalls)
    insidewalls.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
