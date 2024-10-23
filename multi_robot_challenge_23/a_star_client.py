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

        #Variables for the A*-algorithm(Also called Djikstras-algorithm)
        self.a_star_switch = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.arrived_at_goal = False
        self.map_data = None

        #Publish and subscribe to topics
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                                depth=5,
        )
        self.create_subscription(OccupancyGrid, '/map', callback=self.clbk_map, qos_profile=qos_profile)
        self.namespace = self.get_namespace()
        self.cmd_vel_publisher = self.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10)

        #Define service using the handle_service-function to handle service calls.
        self.srv = self.create_service(MoveTarget, 'go_to_point', self.handle_service)



    #Loads the map blueprint and transforms it into a double array instead of a single one.
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

    #Handles calls to the service
    def handle_service(self, request, response):
        self.a_star_switch = request.move_switch
        self.goal_x = request.target_position.x
        self.goal_y = request.target_position.y
        response.success = self.arrived_at_goal
        self.get_logger().info(f'A*-algorithm is set to {self.a_star_switch} for namespace {self.namespace}')
        return response

    #Helper functions
    def heuristic(self, start_x, start_y, goal_x, goal_y):
        return math.sqrt(math.pow(goal_x - start_x, 2) + math.pow(goal_y - start_y, 2))

    #Main 
    def a_star_main(self):
        if(self.a_star_switch):
            openset = set()
