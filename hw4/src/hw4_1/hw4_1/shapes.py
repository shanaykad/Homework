import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
import math, time

class Geometry(Node):
    def __init__(self):
        super().__init__('draw_shapes')
        self.teleport_client = self.create_client(TeleportAbsolute, 'turtlesim4/turtle1/teleport_absolute')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        # while not self.spawn_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('spawn service not available, waiting again...')
        # while not self.kill_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('kill service not available, waiting again...')
        # while not self.teleport_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('teleport service not available, waiting again...')
        # self.spawn_turtle()
            
        self.publisher_tri = self.create_publisher(Twist, 'turtlesim1/turtle1/cmd_vel', 10)
        self.publisher_squ = self.create_publisher(Twist, 'turtlesim2/turtle1/cmd_vel', 10)
        self.publisher_dec = self.create_publisher(Twist, 'turtlesim3/turtle1/cmd_vel', 10)
        self.publisher_cir = self.create_publisher(Twist, 'turtlesim4/turtle1/cmd_vel', 10)

    def teleport_absolute(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        self.teleport_client.call_async(request)

    def spawn_turtle(self, x, y, name):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = name
        self.spawn_client.call_async(request)

    def kill_turtle(self, turtle_name):
        request = Kill.Request()
        request.name = turtle_name
        self.kill_client.call_async(request)

    def draw_triangle(self):
        # self.spawn_turtle(1.0, 8.0, 'triangleturtle')
        msg = Twist()
        i=1
        for i in range (3):
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.publisher_tri.publish(msg)
            time.sleep(3)
            msg.linear.x = 0.0
            msg.angular.z = math.pi/1.5
            self.publisher_tri.publish(msg)
            time.sleep(1)
            i = i + 1
        # self.kill_turtle('triangleturtle')

    def draw_square(self):
        self.spawn_turtle(1.0, 8.0, 'squareturtle')
        msg = Twist()
        i=1
        for i in range (4):
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.publisher_squ.publish(msg)
            time.sleep(4)
            msg.linear.x = 0.0
            msg.angular.z = math.pi/2
            self.publisher_squ.publish(msg)
            time.sleep(1)
            i = i + 1
        self.kill_turtle('squareturtle')
    
    def draw_decagon(self):
        self.spawn_turtle(1.0, 8.0, 'decagonturtle')
        msg = Twist()
        i=1
        for i in range (10):
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.publisher_dec.publish(msg)
            time.sleep(2)
            msg.linear.x = 0.0
            msg.angular.z = math.pi/5
            self.publisher_dec.publish(msg)
            time.sleep(1)
            i = i + 1
        self.kill_turtle('decagonturtle')

    def draw_circle(self):
        self.teleport_absolute(2.5, 6.0, 0.0)
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        self.publisher_cir.publish(msg)
        time.sleep(2*math.pi/msg.angular.z)


def main(args=None):
    rclpy.init(args=args)

    draw_shapes = Geometry()
    # draw_shapes.kill_turtle('turtle1')
    draw_shapes.draw_circle()

    rclpy.spin(draw_shapes)
    draw_shapes.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
