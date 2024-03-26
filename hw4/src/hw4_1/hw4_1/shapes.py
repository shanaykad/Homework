import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

class DrawShapes(Node):
    def __init__(self):
        super().__init__('draw_shapes')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(1.0, self.draw_shapes)
        self.current_shape_index = 0
        self.shapes = [
            ('triangle', [(1, 1), (4, 1), (2.5, 4), (1, 1)]),  # Quadrant 1
            ('square', [(6, 1), (9, 1), (9, 4), (6, 4), (6, 1)]),  # Quadrant 2
            ('decagon', [(1, 6), (3, 7), (5, 8), (7, 9), (9, 9), (11, 9), (13, 8), (15, 7), (17, 6), (19, 6), (1, 6)]),  # Quadrant 3
            ('circle', self.generate_circle_points(15, 15, 2, 20))  # Quadrant 4, adjust circle coordinates and size
        ]

    def pose_callback(self, data):
        # Do nothing with pose
        pass

    def draw_shapes(self):
        if self.current_shape_index >= len(self.shapes):
            self.timer.cancel()
            self.get_logger().info('All shapes drawn.')
            return

        shape_name, points = self.shapes[self.current_shape_index]
        self.get_logger().info(f'Drawing {shape_name}...')

        for point_index in range(len(points) - 1):
            start_point = points[point_index]
            end_point = points[point_index + 1]
            self.draw_line(start_point, end_point)

        self.current_shape_index += 1

    def draw_line(self, start_point, end_point):
        twist = Twist()
        twist.linear.x = float(end_point[0] - start_point[0])
        twist.linear.y = float(end_point[1] - start_point[1])
        distance = math.sqrt(twist.linear.x ** 2 + twist.linear.y ** 2)
        twist.linear.x /= distance
        twist.linear.y /= distance
        twist.angular.z = 0.0  # Ensure angular component is float

        # Publish the twist command
        self.publisher_.publish(twist)
        self.get_logger().info(f'Drawn line from {start_point} to {end_point}')

        # Sleep to give the turtle time to move
        time.sleep(1)  # Adjust the delay as needed (1 second in this example)

    def generate_circle_points(self, center_x, center_y, radius, num_points):
        points = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            points.append((x, y))
        points.append(points[0])  # Close the circle
        return points

def main(args=None):
    rclpy.init(args=args)
    draw_shapes = DrawShapes()
    rclpy.spin(draw_shapes)
    draw_shapes.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
