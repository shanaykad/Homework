import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
import math, numpy, scipy, time

class Geometry(Node):
    def __init__(self):
        super().__init__('draw_shapes')
        self.teleport_client = self.create_client(TeleportAbsolute, 'teleport_abs')
        # while not self.teleport_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('teleport service not available, waiting again...')
            
        self.publisher_ = self.create_publisher(Twist, 'turtlesim1/turtle1/cmd_vel', 10)

    def teleport_absolute(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        self.teleport_client.call_async(request)

    def draw_triangle(self):
        msg = Twist()
        i=1
        for i in range (3):
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            time.sleep(3)
            msg.linear.x = 0.0
            msg.angular.z = math.pi/1.5
            self.publisher_.publish(msg)
            time.sleep(1)
            i = i + 1

    def draw_square(self):
        msg = Twist()
        i=1
        for i in range (4):
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            time.sleep(4)
            msg.linear.x = 0.0
            msg.angular.z = math.pi/2
            self.publisher_.publish(msg)
            time.sleep(1)
            i = i + 1
    
    def draw_decagon(self):
        msg = Twist()
        i=1
        for i in range (10):
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            time.sleep(2)
            msg.linear.x = 0.0
            msg.angular.z = math.pi/5
            self.publisher_.publish(msg)
            time.sleep(1)
            i = i + 1

    def draw_circle(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        time.sleep(2*math.pi/msg.angular.z)


def main(args=None):
    rclpy.init(args=args)

    draw_shapes = Geometry()
    draw_shapes.teleport_absolute(1.0 ,1.0 , 3.14)

    rclpy.spin(draw_shapes)
    draw_shapes.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
