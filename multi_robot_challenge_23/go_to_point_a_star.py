import time
import rclpy
import math
import random

from .a_star import AStar

from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from math import atan2, sqrt, pow
from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool 
from sensor_msgs.msg import LaserScan
from interfaces.srv import SetGoal


class GoToPointController(Node):

    def __init__(self):
        super().__init__('go_to_point')

        # Wait for all other services to be up and running
        time.sleep(15)

        self.declare_parameter('namespace', '')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value        
        self.get_logger().info(f'Namespace: {self.namespace}')
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            f'{self.namespace}/cmd_vel', 
            10)
        self.get_logger().info(f'Publisher created for {self.namespace}/cmd_vel')

        self.map_future = Future()
        self.subscription_odom = self.create_subscription(
            Odometry,
            f'{self.namespace}/odom',
            self.odom_callback,
            10)
        self.get_logger().info(f'Subscription created for {self.namespace}/odom')

        self.lidar_front = 0.0
        self.lidar_sensor = self.create_subscription(
            LaserScan,
            f'{self.namespace}/scan',
            self.lidar_callback,
            10
        )

        #Created a method for the central controller to receive information for when go_to_point_a_star has reached its destination
        self.goal_reached_publisher = self.create_publisher(
            Bool, 
            f'{self.namespace}/goal_reached', 
            10
        )
        self.get_logger().info(f"Publisher created for {self.namespace}/goal_reached")
        
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                                depth=5,)

        self.map_subscription = self.create_subscription(
            OccupancyGrid, 
            '/filtered_map', 
            callback=self.clbk_map, 
            qos_profile=qos_profile)
        self.get_logger().info('Subscription created for /map')

        self.map_data = None
        self.map_info = None
        self.a_star_class = None
        self.path=[]

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.current_goal_x = 0.0
        self.current_goal_y = 0.0

        # Robot position and orientation  b
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0  
        self.desired_yaw_var = 0.0
        
        # The robot should stop within 10cm of the target destination. 
        # When centering on the target, the angle does not have to be 'perfect'
        self.goal_tolerance = 0.1
        self.yaw_tolerance = 0.2

        #Server switch and initialization
        self.go_to_goal_switch = False
        self.srv = self.create_service(SetGoal, f'{self.namespace}/go_to_point_service', self.handle_service)
        self.get_logger().info(f'Service created for {self.namespace}/go_to_point_service')
        print("Service is running for: ", self.namespace)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info('Timer created')

    def clbk_map(self, msg):
        map_width = msg.info.width
        map_height = msg.info.height
        occupancy_grid = msg.data
        grid = [[0 for _ in range(map_width)] for _ in range(map_height)]

        for i in range(map_height):
            for j in range(map_width):
                index = i * map_width + j
                grid[i][j] = occupancy_grid[index]

        self.map_data = grid
        self.map_info = msg.info
        self.a_star_class = AStar(map_height, map_width, grid, msg.info)

        #Since we only need to read the map once, we destroy the subscription
        self.map_subscription.destroy()

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw

    def lidar_callback(self, msg):
        self.lidar_front = min(min(msg.ranges[0:10]), min(msg.ranges[350:359]))

    def has_crashed(self):
        return self.lidar_front < 0.15
    
    def handle_collision(self):
        self.stop_robot()
        self.get_logger().info('Collision detected!')
        twist = Twist()
        twist.linear.x = -0.5
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        time.sleep(2)
        self.stop_robot()
        num = random.randint(1, 15)
        time.sleep(num)
        self.go_to_goal_switch = True


    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def go_to_goal(self, desired_yaw, linear_vel):
        twist = Twist()
        current_yaw = self.normalize_angle(self.yaw)
        desired_yaw = self.normalize_angle(desired_yaw)
        delta_yaw = self.normalize_angle(desired_yaw - current_yaw)
        angular_speed = 1.0 * delta_yaw
        max_angular_speed = 0.4
        twist.angular.z = max(min(angular_speed, max_angular_speed), -max_angular_speed)
        twist.linear.x = linear_vel
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.go_to_goal_switch = False

    def handle_service(self, request, response):
        self.get_logger().info('Request received')
        success = True
        self.go_to_goal_switch = request.move_switch
        self.goal_x = request.target_position.x
        self.goal_y = request.target_position.y
        self.get_logger().info(f'Received goal: ({self.goal_x}, {self.goal_y})')
        current = (self.current_x, self.current_y)
        goal = (self.goal_x, self.goal_y)
        if self.map_data is not None:
            self.path = self.a_star_class.a_star_search(current, goal)
            if self.path is None:
                self.goal_reached_publisher.publish(Bool(data=True))
                self.stop_robot()
                self.go_to_goal_switch = False
            else:
                next_goal = self.path.pop(0)
                self.current_goal_x = next_goal[0]
                self.current_goal_y = next_goal[1]
            response.success = True
        else:
            self.get_logger().error('Map data is None')
            success = False
        response.success = success
        return response
    
    def timer_callback(self):
        if self.go_to_goal_switch:
            self.follow_path()
            if self.has_crashed():
                self.handle_collision()
            

    def follow_path(self):
        if self.isAtCurrentGoal():
            if self.path:
                next_goal = self.path.pop(0)
                self.current_goal_x = next_goal[0]
                self.current_goal_y = next_goal[1]
            else:
                self.get_logger().info('Goal reached! Publishing to subscribers.')

                self.goal_reached_publisher.publish(Bool(data=True))

                self.stop_robot()
                self.go_to_goal_switch = False
                return

        desired_yaw = math.atan2(
            self.current_goal_y - self.current_y,
            self.current_goal_x - self.current_x
        )

        if self.correct_angle(desired_yaw):
            self.go_to_goal(desired_yaw, 0.4) 
        else:
            self.go_to_goal(desired_yaw, 0.0)

    def correct_angle(self, desired_yaw):
        yaw_diff = abs(self.normalize_angle(desired_yaw) - self.normalize_angle(self.yaw))
        return (yaw_diff < self.yaw_tolerance)


    def isAtCurrentGoal(self):
        distance_to_goal = sqrt(pow((self.current_goal_x - self.current_x), 2) + pow((self.current_goal_y - self.current_y), 2))
        return distance_to_goal <= self.goal_tolerance

def main(args=None):
    rclpy.init(args=args)

    controller = GoToPointController()
    
    rclpy.spin(controller)

    controller.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
