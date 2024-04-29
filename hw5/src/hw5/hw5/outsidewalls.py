import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class FollowWalls(Node):

    def __init__(self):
        super().__init__('hw5')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.distances = []
        self.progress = 0 # 0: has started, hasn't recieved lidar data yet, 1: recieved data, move forward until wall, 2: reached wall, turn
        self.initiated = 0 # boolean variable
        self.reachedwall = 0 # boolean variable

        self.declare_parameter('traveldist', 0.5)
        self.traveldist = self.get_parameter('traveldist').value

        self.declare_parameter('direction', -1) # -1 for CW, 1 for CCW
        self.direction = self.get_parameter('direction').value

        if self.direction == -1:
            self.scandirection = 89
        elif self.direction == 1:
            self.scandirection = 269

    def scan_callback(self, msg):
        self.distances = msg.ranges
        # print('left dist ' + str(self.distances[89])) # left
        # print('front dist ' + str(self.distances[0])) # front
        # print('right dist ' + str(self.distances[269])) # right
        # print('diagonal diff ' + str(abs(self.distances[89-30] - self.distances[89+30])))
        # print('diagonal left ' + str(abs(self.distances[89+30])))
        # print('diagonal right ' + str(abs(self.distances[89-30])))
        if len(self.distances) > 10 and self.progress == 0:
            self.progress = 1
        if self.progress == 1:
            self.moveForward()
        if self.progress == 2:
            self.turn()
            

    def moveForward(self):
        print('Moving Forward!')
        msg = Twist()
        msg.linear.x = 0.5
        self.publisher.publish(msg)
        if self.distances[0] < 2: # Slows down when close to wall
            msg.linear.x = 0.2
            self.publisher.publish(msg)
        if abs(self.traveldist-self.distances[0]) < 0.2*self.traveldist: # 20% tolerance
            msg.linear.x = 0.0
            self.publisher.publish(msg)
            self.progress = 2
            self.reachedwall = 1
        if self.reachedwall == 1:
            self.corrector()

    def corrector(self):
        msg =  Twist()

        angvel = 0.2*self.direction
        linvel = 0.5

        if self.distances[0] < 0.2*self.traveldist: # Prevents correction during turning
            pass
        elif self.listMin(self.distances, self.scandirection, 80) < 0.8*self.traveldist: # Corrects when too close to wall
            print('Correcting!')
            msg.angular.z = angvel
            msg.linear.x = linvel
            self.publisher.publish(msg)
            time.sleep(0.5)
            msg.angular.z = -angvel
            msg.linear.x = linvel
            self.publisher.publish(msg)
            time.sleep(0.3)
        elif self.listMin(self.distances, self.scandirection, 80) > 1.2*self.traveldist: # Corrects when too far from wall
            print('Correcting!')
            msg.angular.z = -angvel
            msg.linear.x = linvel
            self.publisher.publish(msg)
            time.sleep(0.5)
            msg.angular.z = angvel
            msg.linear.x = linvel
            self.publisher.publish(msg)
            time.sleep(0.3)        

    def turn(self):
        print('Turning!')
        msg = Twist()
        if self.initiated == 0:
            msg.angular.z = 0.5*self.direction
            self.publisher.publish(msg)
            time.sleep(1)
            self.initiated = 1
        msg.angular.z = 0.3*self.direction
        self.publisher.publish(msg)
        leftavg = self.getAverage(self.distances, self.scandirection+20, 5)
        rightavg = self.getAverage(self.distances, self.scandirection-20, 5)
        if abs(rightavg - leftavg) < 0.2*self.traveldist: # Slows down when turn is close to completion
            msg.angular.z = 0.08*self.direction
            self.publisher.publish(msg)
            if abs(rightavg - leftavg) < self.traveldist*0.02 and self.distances[self.scandirection] < 1.1*self.traveldist: # Ensures turn is complete when tb3 is facing roughly parallel to wall and is an appropriate distance away from the wall (10% tolerance)
                self.progress = 1
                self.initiated = 0

    def getAverage(self, list, middleentry, range): # Average of a portion of a list
        average = sum(list[middleentry-range:middleentry+range]) / (2*range+1)
        return average
    
    def listMin(self, array, mid, range): # Minimum value from a portion of a list
        return min(array[mid - range:mid + range + 1]) 


def main(args=None):
    rclpy.init(args=args)
    insidewalls = FollowWalls()

    rclpy.spin(insidewalls)
    insidewalls.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()