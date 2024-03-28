import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative, SetPen, Spawn, Kill
from turtlesim.msg import Empty
import math, time, random

class TurtleSpawn(Node):
    def __init__(self):
        super().__init__('turtle_spawn')
        self.teleport_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('teleport service not available, waiting again...')

        self.clear_client = self.create_client(Empty, 'clear')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('clear service not available, waiting again...')

        self.reset_client = self.create_client(Empty, 'reset')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset service not available, waiting again...')

        self.setpen_client = self.create_client(SetPen, 'turtle1/set_pen')
        while not self.setpen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('setpen service not available, waiting again...')

        self.spawn_client = self.create_client(Spawn, 'spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')

        self.kill_client = self.create_client(Kill, 'kill')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('kill service not available, waiting again...')
            
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    def spawn_random_turtle(self):
        x = random()
        y = random()
        name = ''
        self.spawn_turtle(x, y, name)


    def teleport_absolute(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        self.teleport_client.call_async(request)

    def setpen(self, toggle):
        request = SetPen.Request()
        request.off = toggle
        self.setpen_client.call_async(request)

    def clear_bg(self):
        request = Empty.Request()
        self.clear_client.call_async(request)

    def reset_turtle(self):
        request = Empty.Request()
        self.reset_client.call_async(request)

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


def main(args=None):
    rclpy.init(args=args)

    turtle_spawner = TurtleSpawn()


    rclpy.spin(turtle_spawner)
    turtle_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
