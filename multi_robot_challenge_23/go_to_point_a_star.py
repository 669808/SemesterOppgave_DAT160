import rclpy
import math
from interfaces.srv import SetGoal
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from math import atan2, sqrt, pow
from tf_transformations import euler_from_quaternion
from .a_star import AStar
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid

class GoToPointController(Node):

    def __init__(self):
        super().__init__('go_to_point')
        self.declare_parameter('namespace')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            f'{self.namespace}/cmd_vel', 
            10)

        self.subscription_odom = self.create_subscription(
            Odometry,
            f'{self.namespace}/odom',
            self.odom_callback,
            10)
        
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                                depth=5,)

        self.create_subscription(
            OccupancyGrid, 
            '/map', 
            callback=self.clbk_map, 
            qos_profile=qos_profile)

        self.map_data = None
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
        
        self.arrived_at_goal = False

        #Server switch and initialization
        self.go_to_goal_switch = False
        self.srv = self.create_service(SetGoal, f'{self.namespace}_go_to_point_service', self.handle_service)
        print("Service is running for: ", self.namespace)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback) 

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
        self.a_star_class = AStar(map_height, map_width)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def go_to_goal(self, desired_yaw, linear_vel):
        twist = Twist()
        current_yaw = self.normalize_angle(self.yaw)
        desired_yaw = self.normalize_angle(desired_yaw)
        delta_yaw = self.normalize_angle(desired_yaw - current_yaw)
        angular_speed = 1.0 * delta_yaw
        max_angular_speed = 0.3
        twist.angular.z = max(min(angular_speed, max_angular_speed), -max_angular_speed)
        twist.linear.x = linear_vel
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.arrived_at_goal = True

    def handle_service(self, request, response):
        success = True
        self.go_to_goal_switch = request.move_switch
        self.arrived_at_goal = False
        self.goal_x = request.target_position.x
        self.goal_y = request.target_position.y
        current = (self.current_x, self.current_y)
        goal = (self.goal_x, self.goal_y)
        if self.map_data is not None:
            self.path = self.a_star_class.a_star_search(self.map_data, current, goal)
            next_goal = self.path.pop(0)
            self.current_goal_x = next_goal[0]
            self.current_goal_y = next_goal[1]
        else:
            success = False
        response.success = success
        print(f'Go-to-point switch has been set to {self.go_to_goal_switch}')
        return response

    def timer_callback(self):
        if(self.go_to_goal_switch):
            distance_to_goal = sqrt(pow((self.current_goal_x - self.current_x), 2) + pow((self.current_goal_y - self.current_y), 2))
            desired_yaw = math.atan2(
                        self.current_goal_y - self.current_y,
                        self.current_goal_x - self.current_x)

            if distance_to_goal <= self.goal_tolerance:

                if self.path:
                    next_goal = self.path.pop(0)
                    self.current_goal_x = next_goal[0]
                    self.current_goal_y = next_goal[1]
                else:
                    self.get_logger().info('Goal reached!')
                    self.stop_robot()
                    self.go_to_goal_switch = False
                    return
                
            if (abs(self.normalize_angle(desired_yaw)-self.normalize_angle(self.yaw)) > self.yaw_tolerance):
                self.go_to_goal(desired_yaw, 0.0)
            else:
                self.go_to_goal(desired_yaw, min(0.3*distance_to_goal, 0.3))

def main(args=None):
    rclpy.init(args=args)
    controller = GoToPointController()
    
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
