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
        if not self.map_adjusted and self.map_data is not None:
            # TODO: Take the map read from the '/map'-topic and convert it to a frontier map.
            # We have to convert all cells that have no obstacles to take the value -1, so we have a unexplore map.
            for i in range(len(self.map_data)):
                for j in range(len(self.map_data[0])):
                    if self.map_data[i][j] == 0:
                        self.map_data[i][j] = -1
            self.map_adjusted = True
    
    def clbk_map(self, msg):
        if self.map_data is None:
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
        nearest_frontier = None
        min_distance = float('inf')
        for frontier in self.frontiers:
            distance = ((robot_pos[0] - frontier[0]) ** 2 + (robot_pos[1] - frontier[1]) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                nearest_frontier = frontier
        return nearest_frontier
        #If there are no frontiers available, the return value will be None
        

    def update_frontier_map(self, data, robot_current_pos):
        success = True
        # TODO: Take the data from the robot and update the frontier map. The existing frontier that was reached
        # needs to be removed and the new frontiers need to be calculated.
        
        
        return success

      
    def find_frontiers(self):
        frontiers = []
        map_data = self.map_data
        visited = [[False for _ in range(len(map_data[0]))] for _ in range(len(map_data))]

        for i in range(len(map_data)):
            for j in range(len(map_data[0])):
                if map_data[i][j] == -1 and self.is_border_cell(map_data, i, j) and not visited[i][j]:
                    frontier_group = self.flood_fill(map_data, visited, i, j)
                    if frontier_group:
                        centroid = self.calculate_centroid(frontier_group)
                        frontiers.append(centroid)
        
        self.frontiers = frontiers
        return frontiers

    def is_border_cell(self, map_data, x, y):
        neighbors = [
            (x-1, y), (x+1, y), 
            (x, y-1), (x, y+1)
        ]
        
        for nx, ny in neighbors:
            if 0 <= nx < len(map_data) and 0 <= ny < len(map_data[0]):
                if map_data[nx][ny] != -1:
                    return True
        return False

    def flood_fill(self, map_data, visited, x, y):
        frontier_group = []
        stack = [(x, y)]
        
        while stack:
            cx, cy = stack.pop()
            if visited[cx][cy]:
                continue
            visited[cx][cy] = True
            frontier_group.append((cx, cy))
            
            neighbors = [
                (cx-1, cy), (cx+1, cy), 
                (cx, cy-1), (cx, cy+1)
            ]
            
            for nx, ny in neighbors:
                if 0 <= nx < len(map_data) and 0 <= ny < len(map_data[0]):
                    if map_data[nx][ny] == -1 and self.is_border_cell(map_data, nx, ny) and not visited[nx][ny]:
                        stack.append((nx, ny))
        
        return frontier_group

    def calculate_centroid(self, frontier_group):
        x_sum = sum([x for x, _ in frontier_group])
        y_sum = sum([y for _, y in frontier_group])
        count = len(frontier_group)
        return (x_sum // count, y_sum // count)

def main(args=None):
    rclpy.init(args=args)
    FBS = FrontierBasedSearch()
    
    rclpy.spin(FBS)

    FBS.destroy_node()
    rclpy.shutdown()[[]]

if __name__ == '__main__':
    main()


