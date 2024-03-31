import rclpy, math, time, random
from rclpy.node import Node
from turtlesim.srv import SetPen, Spawn, Kill
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray


class TurtleSpawn(Node):
    def __init__(self):
        super().__init__('turtle_spawn')

        self.create_timer(10, self.timed_spawner)

        self.arrpublisher_ = self.create_publisher(Float64MultiArray, 'alive_turtles', 2)
        # self.arrpublisher_ = self.create_publisher(Float64MultiArray, 'turtlearraysptoco', 2)
        # self.arrsubscriber_ = self.create_subscription(Float64MultiArray, 'turtlearraycotosp', self.subscriber_callback, 2)
        self.create_timer(0.5, self.timed_publisher)
        self.arr = []
        self.currentid = 0



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

    def spawn_random_turtle(self, amount):
        for i in range(amount):
            x = random.uniform(0.5, 10.58)
            y = random.uniform(0.5, 10.58)
            self.spawn_turtle(x, y, self.currentid)
            self.currentid = self.currentid + 1

    def timed_spawner(self):
        self.spawn_random_turtle(1)

    def timed_publisher(self):
        msg = Float64MultiArray()
        msg.data = self.arr
        self.arrpublisher_.publish(msg)

    def subscriber_callback(self, msg):
        self.arr = msg.data



    def setpen(self, toggle):
        request = SetPen.Request()
        request.off = toggle
        self.setpen_client.call_async(request)

    def reset_turtle(self):
        request = Empty.Request()
        self.reset_client.call_async(request)

    def spawn_turtle(self, x, y, id):
        self.arr.append(x)
        self.arr.append(y)
        self.arr.append(float(id))
        print(self.arr)
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = 'targetturtle' + str(id)
        self.spawn_client.call_async(request)

    def kill_turtle(self, turtle_name):
        request = Kill.Request()
        request.name = turtle_name
        self.kill_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    turtle_spawner = TurtleSpawn()

    turtle_spawner.spawn_random_turtle(3) # was 4

    rclpy.spin(turtle_spawner)
    turtle_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
