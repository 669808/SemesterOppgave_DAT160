import rclpy
from rclpy import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from interfaces.srv import FrontierRequest


class FrontierBasedSearch(Node):
    def __init__(self):
        super().__init__('frontier_based_search')

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
        self.map = None
        self.map_adjusted = False

        self.frontier_list = []

        self.srv = self.create_service(FrontierRequest, 'frontier_based_search', self.handle_service)

    def create_frontier_map(self):
        if not self.map_adjusted:
            x = 1
            # TODO: Take the map read from the '/map'-topic and convert it to a frontier map.
            # We have to convert all cells that have no obstacles to take the value -1, so we have a unexplore map.
    
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

    def handle_service(self, request, response):
        # TODO: Each time a request is sent, this service should recieve sensor data to update the map in addition to
        # the position of the robot. The map is updated and a new frontier is returned
        response.success = self.update_frontier_map(request.data, request.robot_pos)
        if response.success:
            response.frontier = self.find_nearest_frontier(request.robot_pos)
        return response

    # Method to find the nearest frontier of a robot
    def find_nearest_frontier(self, robot_pos):
        return 
    
    def update_frontier_map(self, data, robot_current_pos):
        success = True
        # TODO: Take the data from the robot and update the frontier map. The existing frontier that was reached
        # needs to be removed and the new frontiers need to be calculated.
        return success

    
def main(args=None):
    rclpy.init(args=args)
    FBS = FrontierBasedSearch()
    
    rclpy.spin(FBS)

    FBS.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


