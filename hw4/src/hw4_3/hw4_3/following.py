import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative, SetPen
import math, time

class SingleTurtle(Node):
    def __init__(self):
        super().__init__('')
        self.teleport_abs_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.teleport_abs_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('abs teleport service not available, waiting again...')

        self.teleport_rel_client = self.create_client(TeleportRelative, 'turtle1/teleport_relative')
        while not self.teleport_rel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('rel teleport service not available, waiting again...')

        self.setpen_client = self.create_client(SetPen, 'turtle1/set_pen')
        while not self.setpen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('setpen service not available, waiting again...')
            
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    def teleport_absolute(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        self.teleport_abs_client.call_async(request)

    def teleport_relative(self, linear, angular):
        request = TeleportRelative.Request()
        request.linear = linear
        request.angular = angular
        self.teleport_rel_client.call_async(request)

    def setpen(self, toggle):
        request = SetPen.Request()
        request.off = toggle
        self.setpen_client.call_async(request)

    def draw_triangle(self):
        self.setpen(1)
        self.teleport_absolute(1.0, 6.5, 0.0)
        self.setpen(0)
        msg = Twist()
        i=1
        for i in range (3):
            msg.linear.x = 3.0
            self.publisher_.publish(msg)
            time.sleep(2)
            self.teleport_relative(0.0,2*math.pi/3)
            i = i + 1

    def draw_square(self):
        self.setpen(1)
        self.teleport_absolute(6.5,6.5,0.0)
        self.setpen(0)
        msg = Twist()
        i=1
        for i in range (4):
            msg.linear.x = 3.0
            msg.linear.y = 0.0
            self.publisher_.publish(msg)
            time.sleep(1)
            self.teleport_relative(0.0,math.pi/2)
            i = i + 1
    
    def draw_decagon(self):
        self.setpen(1)
        self.teleport_absolute(2.5, 1.0, 0.0)
        self.setpen(0)
        msg = Twist()
        i=1
        for i in range (10):
            msg.linear.x = 2.0
            self.publisher_.publish(msg)
            time.sleep(0.5)
            self.teleport_relative(0.0,math.pi/5)
            i = i + 1

    def draw_circle(self):
        self.setpen(1)
        self.teleport_absolute(8.0, 1.0, 0.0)
        self.setpen(0)
        msg = Twist()
        msg.linear.x = 3*math.pi
        msg.angular.z = 2*math.pi
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    draw_shapes = SingleTurtle()
    draw_shapes.draw_triangle()
    draw_shapes.draw_square()
    draw_shapes.draw_decagon()
    draw_shapes.draw_circle()

    rclpy.spin(draw_shapes)
    draw_shapes.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
