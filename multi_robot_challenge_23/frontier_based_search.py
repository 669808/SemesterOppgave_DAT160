import rclpy
from rclpy import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from interfaces.srv import FrontierRequest
from collections import deque


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

    #Updated the create_frontier map
    def create_frontier_map(self):
        if self.map_data is None:
            return

        #Iterate over the map data to create a frontier map
        frontier_map = []
        for i in range(len(self.map_data)):
            for j in range(len(self.map_data[i])):
                if self.map_data[i][j] == -1:  # If the cell is unexplored
                    #Check if it borders a known space (value >= 0)
                    if any(self.map_data[ni][nj] >= 0 for ni, nj in self.get_neighbors(i, j)):
                        frontier_map.append((i, j))

        self.frontier_list = frontier_map
        self.map_adjusted = True

    
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

    #defined a get_neighbour function
    def get_neighbours(self, x, y):
        neighbors = [
         (x - 1, y), (x + 1, y), 
            (x, y - 1), (x, y + 1)
     ]
        #Filter out neighbors that are out of bounds
        return [(nx, ny) for nx, ny in neighbors if 0 <= nx < len(self.map_data) and 0 <= ny < len(self.map_data[0])]



    def handle_service(self, request, response):
        # TODO: Each time a request is sent, this service should recieve sensor data to update the map in addition to
        # the position of the robot. The map is updated and a new frontier is returned
        response.success = self.update_frontier_map(request.data, request.robot_pos)
        if response.success:
            response.frontier = self.find_nearest_frontier(request.robot_pos)
        return response

    #Method to find the nearest frontier of a robot
    def find_nearest_frontier(self, robot_pos):
        return 
    
    def update_frontier_map(self, data, robot_current_pos):
        success = True

        #Update map data with the latest sensor data
        #TODO add logic to process 'data' and integrate it into self.map_data)
        for sensor_reading in data:
            
            x_offset, y_offset, occupancy_value = sensor_reading

            #Calculate absolute map position based on the robot's current position
            map_x = robot_current_pos[0] + x_offset
            map_y = robot_current_pos[1] + y_offset

            #Ensure the calculated position is within the map bounds
            if 0 <= map_x < len(self.map_data) and 0 <= map_y < len(self.map_data[0]):
                self.map_data[map_x][map_y] = occupancy_value

        #Remove the reached frontier
        self.frontier_list = [f for f in self.frontier_list if f != robot_current_pos]

        #recalculate frontiers
        self.create_frontier_map()

        return success


    def find_nearest_frontier(self, robot_pos):
        if not self.frontier_list:
            return None

        #Use a BFS approach to find the nearest frontier
        queue = deque([robot_pos])
        visited = set()
        visited.add(robot_pos)

        while queue:
            current_pos = queue.popleft()
            if current_pos in self.frontier_list:
                return current_pos

            for neighbor in self.get_neighbors(*current_pos):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        return None

    
def main(args=None):
    rclpy.init(args=args)
    FBS = FrontierBasedSearch()
    
    rclpy.spin(FBS)

    FBS.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


