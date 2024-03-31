import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, Spawn, Kill, TeleportAbsolute
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray

class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        self.posesubscriber_ = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.arrsubscriber_ = self.create_subscription(Float64MultiArray, 'alive_turtles', self.array_callback, 2)
        # self.arrsubscriber_ = self.create_subscription(Float64MultiArray, 'turtlearraysptoco', self.array_callback, 2)
        # self.arrpublisher_ = self.create_publisher(Float64MultiArray, 'turtlearraycotosp', 2)
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        self.arr = [0.0, 0.0]
        self.x = 5.54
        self.y = 5.54
        self.theta = 0.0
        self.current = 0
        self.distance = 100

        self.declare_parameter('avoidwalls', 0)
        self.avoidwalls = self.get_parameter('avoidwalls').value

        self.seekertimer = self.create_timer(0.5, self.turtle_seeker)
        self.disttimer = self.create_timer(0.2, self.goal_distance)
        self.dirtimer = self.create_timer(0.2, self.goal_direction)
        self.wallstimer = self.create_timer(0.1, self.avoid_walls)
        # self.create_timer(0.5, self.turtle_killer)



        self.reset_client = self.create_client(Empty, 'reset')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset service not available, waiting again...')

        self.setpen_client = self.create_client(SetPen, 'turtle1/set_pen')
        while not self.setpen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('setpen service not available, waiting again...')

        self.teleport_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('teleport service not available, waiting again...')

        self.spawn_client = self.create_client(Spawn, 'spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')

        self.kill_client = self.create_client(Kill, 'kill')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('kill service not available, waiting again...')

    
    def pose_callback(self, msg): # retrieves position info
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.avoidwalls = self.get_parameter('avoidwalls').value
    
    def array_callback(self, msg): # retrieves array
        self.arr = list(msg.data)

    def turtle_seeker(self): # checks if turtle has found target
        print(self.current)
        if self.current not in self.arr:
            return
        if self.distance < 0.8:
            name = 'targetturtle' + str(self.current)
            self.kill_turtle(name)
            self.current = self.current + 1
            self.distance = 100
            # print('array: ' + str(self.arr))

    # def turtle_seeker_bonus(self):
    #     activetargets = int(len(self.arr)/3)
    #     print('there are currently ' + str(activetargets) + ' targets')
    #     distances = list()
    #     i = 0
    #     for i in range(activetargets):
    #         distances.append(math.sqrt((self.arr[3*i]-self.x)**2 + (self.arr[3*i+1]-self.y)**2))
    #         i += 1
    #     # print(distances)
    #     target = distances.index(min(distances))
    #     print('the closest turtle is ' + str(target))

    #     if distances[target] < 0.8:
    #         name = 'targetturtle' + str(target)
    #         self.kill_turtle(name)
    #         for j in range(3):
    #             del self.arr[3*target]
    #         msg = Float64MultiArray()
    #         msg.data = self.arr
    #         self.arrpublisher_.publish(msg)
    #     print('array: ' + str(self.arr))

    def goal_distance(self): # finds euclidian distance to target
        if self.current not in self.arr:
            return
        self.distance = math.sqrt((self.arr[3*self.current + 0]-self.x)**2 + (self.arr[3*self.current + 1]-self.y)**2)
        # print('goal dist: ' + str(self.distance))

    def goal_direction(self): # finds angle in radians to target (like unit circle)
        if self.current not in self.arr:
            return
        self.direction = math.atan2(self.arr[3*self.current + 1]-self.y, self.arr[3*self.current + 0]-self.x) # in radians
        # print('goal ang: ' + str(self.direction))

        msg = Twist()
        msg.linear.x = 1.0
        if abs(self.direction-self.theta) > 0.2:
            msg.angular.z = 0.5
        else:
            msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def avoid_walls(self):
        if self.avoidwalls == 0:
            return
        if self.x < 0.05 or self.x > 11.03 or self.y < 0.05 or self.y > 11.03:
            self.teleport_absolute(5.44,5.44,0.0)

    def setpen(self, toggle):
        request = SetPen.Request()
        request.off = toggle
        self.setpen_client.call_async(request)

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

    def teleport_absolute(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        self.teleport_client.call_async(request)

    def kill_turtle(self, turtle_name):
        request = Kill.Request()
        request.name = turtle_name
        self.kill_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleControl()
    turtle_controller.setpen(1)


    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
