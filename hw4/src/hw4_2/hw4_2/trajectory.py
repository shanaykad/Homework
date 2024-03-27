import rclpy, math, numpy, scipy, time
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
from std_msgs.msg import String

class DrawFunction(Node):
    def __init__(self):
        super().__init__('draw_function')
        self.teleport_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        self.clear_client = self.create_client(Empty, 'clear')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('teleport service not available, waiting again...')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('clear service not available, waiting again...')
        # self.spawn_turtle()
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Pose, 'turtle1/pose', self.callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback(self, msg):
        # print(msg.x)
        # print(msg.y)
        self.x = msg.x - 5.444
        self.y = msg.y - 5.444

    def timer_callback(self):
        msg = Twist()
        # print(self.vx)
        # print(self.vy)
        self.vx = 1.0
        print(self.func)
        if self.func == (lambda x: x):
            self.vy = 1.0
        elif self.func == (lambda x: x**3):
            self.vy = lambda x: 3*x**2
        elif self.func == (lambda x: math.sin(x)):
            self.vy = math.cos(self.x)
        self.vy = 1.0
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        self.publisher_.publish(msg)


    def draw_function(self, func):
        x = -5.544
        y = func(x) + 5.544
        self.func = func
        self.teleport_absolute(x, y, 0.0)

    def teleport_absolute(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        self.teleport_client.call_async(request)

    def clear_bg(self):
        request = Empty.Request()
        self.clear_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    draw_func = DrawFunction()
    draw_func.teleport_absolute(0.0, 0.0, 0.0)
    draw_func.clear_bg()
    draw_func.draw_function(lambda x: x)
    # draw_func.teleport_absolute(0.0, 0.0, 0.0)
    # draw_func.clear_bg()
    # draw_func.draw_function(lambda x: x**3)
    # draw_func.teleport_absolute(0.0, 0.0, 0.0)
    # draw_func.clear_bg()
    # draw_func.draw_function(lambda x: math.sin(x))

    rclpy.spin(draw_func)
    draw_func.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
