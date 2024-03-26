import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill
import math, numpy, scipy

class DrawFunction(Node):
    def __init__(self):
        super().__init__('draw_function')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('kill service not available, waiting again...')
        # self.spawn_turtle()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self): #AND HERE
        msg = Twist()
        msg.linear.x = numpy.atan2()
        msg.linear.y = numpy.atan2()

    def spawn_turtle(self, x, y):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = 'turtle'
        self.spawn_client.call_async(request)

    def kill_turtle(self, turtle_name):
        request = Kill.Request()
        request.name = turtle_name
        self.kill_client.call_async(request)

    def draw_function(self, func):
        x = -5.44
        y = func(x)
        self.spawn_turtle(x,y)
        vx = scipy.derivative(func)(x) #LEFT OFF HERE

def main(args=None):
    rclpy.init(args=args)

    draw_func = DrawFunction()
    draw_func.kill_turtle('turtle1')
    draw_func.draw_function(lambda x: math.sin(x))  # Example: Drawing a parabola function y = x^2

    rclpy.spin(draw_func)
    draw_func.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
