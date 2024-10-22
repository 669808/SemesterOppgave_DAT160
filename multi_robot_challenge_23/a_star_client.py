import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from interfaces.srv import MoveTarget
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid

class AStar(Node):
    def __init__(self):
        super().__init__('a_star_client')

        #Publish and subscribe to topics
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                                depth=5,
        )
        self.create_subscription(OccupancyGrid, '/map', callback=self.clbk_map, qos_profile=qos_profile)
        namespace = self.get_namespace()
        self.cmd_vel_publisher = self.create_publisher(Twist, f'{namespace}/cmd_vel', 10)

        #Define service using the handle_service-function to handle service calls.
        self.srv = self.create_service(MoveTarget, 'go_to_point', self.handle_service)

        #Variabled for the A*-algorithm(Also called Djikstras-algorithm)
        self.a_star_switch = False
        self.goal_x = 0.0
        self.goal_y = 0.0

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def handle_service(self, request, response):
        self.a_star_switch = request.move_switch
        self.goal_x = request.target_position.x
        self.goal_y = request.target_position.y
        response.success = self.arrived_at_goal
        self.get_logger().info(f'Go-to-point switch has been set to {self.switch}')
        return response


    def timer_callback(self):
        #TODO: Implement A-star algorithm
        print('Method not implemented')