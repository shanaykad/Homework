import rclpy, math, numpy, scipy, time
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen, Spawn, Kill
from std_srvs.srv import Empty

class DrawFunction(Node):
    def __init__(self):
        super().__init__('draw_function')
        
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
        self.subscriber_ = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        
        self.timer1 = self.create_timer(0.1, self.linear_timer_callback)
        self.timer2 = self.create_timer(0.1, self.cubic_timer_callback)
        self.timer3 = self.create_timer(0.1, self.sine_timer_callback)

        self.active_function = 'None'
        self.x = 0.0
        self.y = 0.0
        self.k = 1.0
        self.w = 1.0

    def pose_callback(self, msg):
        self.x = msg.x - 5.444
        self.y = msg.y - 5.444

    def linear_timer_callback(self):
        if not self.active_function == 'Linear':
            return
        msg = Twist()
        msg.linear.x = 1.0
        msg.linear.y = self.k
        self.publisher_.publish(msg)
        if self.x > 5.5:
                self.reset_turtle()
                self.active_function == 'None'
                self.x = 0.0
                self.draw_function('Cubic')

    def cubic_timer_callback(self):
        if not self.active_function == 'Cubic':
            return
        msg = Twist()
        self.vx = 1.0
        self.vy = self.k*3*self.x**2
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        if self.x < (-1*(5.544/self.k)**(1./3)):
            msg.linear.y = 0.0
        self.publisher_.publish(msg)
        if self.x > 5.5:
                self.reset_turtle()
                self.active_function == 'None'
                self.x = 0.0
                self.draw_function('Sine')
        
    def sine_timer_callback(self):
        if not self.active_function == 'Sine':
                return
        if self.active_function == 'Sine':
            msg = Twist()
            self.vx = 1.0
            self.vy = self.k*self.w*math.cos(self.w*self.x)
            msg.linear.x = self.vx
            msg.linear.y = self.vy
            self.publisher_.publish(msg)
            if self.x > 5.5:
                self.active_function == 'None'
                self.x = 0.0
                self.destroy_node()
                rclpy.shutdown()


    def draw_function(self, function_name):
        
        if function_name == 'Linear':
            func = lambda x: self.k*x
            x = -5.544/self.k
            y = func(x) + 5.544
            time.sleep(1)
            self.setpen(1)
            self.teleport_absolute(x + 5.544, 0.0, 0.0)
            self.setpen(0)
            self.active_function = 'Linear'

        elif function_name == 'Cubic':
            func = lambda x: self.k*x**3
            x = -1*(5.544/self.k)**(1./3)
            y = func(x) + 5.544
            # print('initial x: ' + str(x + 5.544))
            time.sleep(1)
            self.setpen(1)
            self.teleport_absolute(0.0 , 0.0, 0.0)
            self.setpen(0)
            self.active_function = 'Cubic'
            
        elif function_name == 'Sine':
            func = lambda x: self.k*math.sin(self.w*x)
            x = -5.544
            y = func(x) + 5.544
            time.sleep(1)
            self.setpen(1)
            self.teleport_absolute(x + 5.544, y, 0.0)
            self.setpen(0)
            self.active_function = 'Sine'

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

    draw_func = DrawFunction()

    draw_func.draw_function('Linear')

    rclpy.spin(draw_func)
    draw_func.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
