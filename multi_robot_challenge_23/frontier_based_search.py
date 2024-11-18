import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
#I do not have this file in my directory, therefore there's an error
from interfaces.srv import FrontierRequest
from collections import deque
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.colors as mcolors
import time
import matplotlib.pyplot as plt
#Import LIDAR
from sensor_msgs.msg import LaserScan
import math

#Implemented LIDAR. Map gets updated in real time, takes a lot of computional power. Functions on my machine.

class FrontierBasedSearch(Node):
    def __init__(self):
        super().__init__('frontier_based_search')

        self.map_data = None
        self.map_info = None
        self.map = None
        self.map_adjusted = False

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                                depth=5,)

        self.map_recieved = False

        self.map_subscription = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            callback=self.clbk_map, 
            qos_profile=qos_profile)

        self.frontier_centroids = []

        self.frontier_list = []

        self.create_subscription(
            Odometry,
            'tb3_0/odom',
            self.odom_callback_tb3_0,
            10
        )

        self.create_subscription(
            Odometry,
            'tb3_1/odom',
            self.odom_callback_tb3_1,
            10
        )

        self.robot_pos_tb3_0 = (0, -1.0)
        self.robot_pos_tb3_1 = (0, 1.0)

        self.latest_lidar_data_tb3_0 = None
        self.latest_lidar_data_tb3_1 = None

        self.create_subscription(
            LaserScan,
            'tb3_0/scan',
            self.lidar_callback_tb3_0,
            QoSProfile(depth=10)  #Adjust depth as needed
        )

        self.create_subscription(
            LaserScan,
            'tb3_1/scan',
            self.lidar_callback_tb3_1,
            QoSProfile(depth=10)  #Adjust depth as needed
        )

        self.create_timer(0.2, self.proccess_lidar_readings)

        self.fbs_service = self.create_service(
            FrontierRequest, 
            'frontier_based_search', 
            self.handle_service
            )
        
        self.get_logger().info('Service is running')

    def handle_service(self, request, response):
        self.get_logger().info('Service request received')
        robot_position = self.get_map_coords((request.current_pos.x, request.current_pos.y))
        response.success = self.update_frontier_map(robot_position)
        self.visualize_map()
        if response.success:
            frontier = self.find_nearest_frontier(robot_position)
            if frontier is not None:
                point = Point()
                point.x, point.y = self.get_world_pos(frontier[0], frontier[1])
                response.frontier = point
        return response
    
    def odom_callback_tb3_0(self, msg):
        world_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.robot_pos_tb3_0 = world_pos

    def odom_callback_tb3_1(self, msg):
        world_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.robot_pos_tb3_1 = world_pos
        

    def clbk_map(self, msg):
        self.get_logger().info("Map data proccessing")
        map_width = msg.info.width
        map_height = msg.info.height
        occupancy_grid = msg.data
        grid = [[0 for _ in range(map_width)] for _ in range(map_height)]

        for i in range(map_height):
            for j in range(map_width):
                index = i * map_width + j
                if occupancy_grid[index] == 0:
                    grid[i][j] = -1
                else:
                    grid[i][j] = occupancy_grid[index]

        grid = np.array(grid)
        self.map_data = grid
        self.map_info = msg.info
        self.map_recieved = True
        self.get_logger().info("Map data received")
        self.map_subscription.destroy()

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
  
    #defined a get_neighbour function
    def get_neighbors (self, x, y):
        neighbors = [
         (x - 1, y), (x + 1, y), 
            (x, y - 1), (x, y + 1)
        ]
        #Filter out neighbors that are out of bounds
        return [(nx, ny) for nx, ny in neighbors if 0 <= nx < len(self.map_data) and 0 <= ny < len(self.map_data[0])]

    def update_frontier_map(self, robot_current_pos):
        if self.map_data is None:
            self.get_logger().warn("Map data is not initialized.")
            return False
        
        frontier_cells = self.find_frontier_cells(robot_current_pos)

        if not frontier_cells:
            self.get_logger().warn("No frontier cells found.")
            return False

        centroids = self.find_centroids(frontier_cells)

        for centroid in self.frontier_centroids:
            cx, cy = centroid
            #If the centroids found in find_centroids are within the 100 cells of the 
            #centroids already existing, then we do not add them to self.frontier_centroids
            #This is to prevent the same frontier from being added multiple times
            for x, y in centroids:
                if not (abs(x - cx) < 100 and abs(y - cy) < 100):
                    self.frontier_centroids.append((x, y))

    def visualize_map(self):
        # Create a custom colormap
        cmap = mcolors.ListedColormap(['red', 'gray', 'white'])
        bounds = [-1.5, -0.5, 50, 100]
        norm = mcolors.BoundaryNorm(bounds, cmap.N)

        # Convert map_data to a NumPy array for easier manipulation
        map_array = np.array(self.map_data)

        # Plot the map
        plt.imshow(map_array, cmap=cmap, norm=norm)
        plt.colorbar()
        plt.show()

    def find_frontier_cells(self, robot_current_pos):
        queue = deque([robot_current_pos])
        visited = set([robot_current_pos])
        frontier_cells = []

        while queue:
            x, y = queue.popleft()

            for nx, ny in self.get_neighbors(x, y):
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    if self.map_data[nx, ny] == 0:  # Continue BFS within the circle
                        queue.append((nx, ny))
                    elif self.map_data[nx, ny] == -1:  # Potential border cell
                        # Check if any neighbor is within the circle (value 0)
                        if any(self.map_data[bx, by] == 0 for bx, by in self.get_neighbors(nx, ny)):
                            frontier_cells.append((nx, ny))
        
        return frontier_cells

    def find_centroids(self, frontier_list):
        visited = set()
        centroids = []

        def bfs_find_cluster(start):
            queue = deque([start])
            cluster = []
            visited.add(start)
            
            while queue:
                x, y = queue.popleft()
                cluster.append((x, y))
                
                for nx, ny in self.get_neighbors(x, y):
                    if (nx, ny) not in visited and (nx, ny) in frontier_list and self.grid[nx, ny] == -1:
                        visited.add((nx, ny))
                        queue.append((nx, ny))
            
            return cluster
        
        for cell in frontier_list:
            if cell not in visited:
                cluster = bfs_find_cluster(cell)
                if cluster:
                    # Calculate the centroid of the cluster
                    cx = sum(x for x, y in cluster) / len(cluster)
                    cy = sum(y for x, y in cluster) / len(cluster)
                    centroids.append((cx, cy))
    
        return centroids

    def find_nearest_frontier(self, robot_pos):
        if not self.frontier_list:
            return None

        robot_pos = self.get_map_coords(robot_pos)

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
    
    def proccess_lidar_readings(self):
        if not self.map_recieved:
            self.get_logger().info("Map has not been adjusted")
            return
        if self.latest_lidar_data_tb3_0 is not None:
            robot_x, robot_y = self.get_map_coords(self.robot_pos_tb3_0)
            self.lidar_callback(self.latest_lidar_data_tb3_0, robot_x, robot_y)
        if self.latest_lidar_data_tb3_1 is not None:
            robot_x, robot_y = self.get_map_coords(self.robot_pos_tb3_1)
            self.lidar_callback(self.latest_lidar_data_tb3_1, robot_x, robot_y)
    
    def lidar_callback_tb3_0(self, msg: LaserScan):
        self.latest_lidar_data_tb3_0 = msg

    def lidar_callback_tb3_1(self, msg: LaserScan):
        self.latest_lidar_data_tb3_1 = msg

    #LIDAR callback function for processing scan data
    def lidar_callback(self, msg: LaserScan, robot_x, robot_y):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)

        # Update the grid for each lidar point
        for angle, distance in zip(angles, ranges):
            if distance < msg.range_min or distance > msg.range_max:
                distance = msg.range_max

            x = distance * math.cos(angle) * 100
            y = distance * math.sin(angle) * 100

            grid_x = int(robot_x + x)
            grid_y = int(robot_y + y)

            if 0 <= grid_x < self.map_info.width and 0 <= grid_y < self.map_info.height:
                
                if self.map_data[grid_x, grid_y] == -1:
                    self.map_data[grid_x, grid_y] = 0

                self.fill_open_cells(robot_x, robot_y, grid_x, grid_y)

    # Bresenham's line algorithm 
    # The method draws a line between two points and fills 
    # all cells that are on the line. In addition, it fills the cells' neighbors to
    # fill gaps in the sensor readings.
    def fill_open_cells(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while x0 != x1 or y0 != y1:
            if 0 <= x0 < self.map_info.width and 0 <= y0 < self.map_info.height:
                if self.map_data[x0, y0] == -1: 
                    self.map_data[x0, y0] = 0 
                    neighbors = self.get_neighbors_cluster(x0, y0)
                    for nx, ny in neighbors:
                        if self.map_data[nx, ny] == -1:
                            self.map_data[nx, ny] = 0

            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def get_neighbors_cluster (self, x, y):
        neighbors = [
            (x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1),
            (x - 1, y - 1), (x - 1, y + 1), (x + 1, y - 1), (x + 1, y + 1)
        ]
        return neighbors

    #Takes the row and column of the occupancy self.grid and returns a "real world" point represented by x and y.
    def get_world_pos(self, x, y):
        x_pos = round(self.map_info.origin.position.x + y*self.map_info.resolution, 2)
        y_pos = round(self.map_info.origin.position.y + x*self.map_info.resolution, 2)
        world_pos = (x_pos, y_pos)
        return world_pos
    
    #Takes a point and converts it into corresponding row and column in the occupancy self.grid
    def get_map_coords(self, point):
        x = int((point[1] - self.map_info.origin.position.y) / self.map_info.resolution)
        y = int((point[0] - self.map_info.origin.position.x) / self.map_info.resolution)
        return (x, y)

def main(args=None):
    rclpy.init(args=args)
    FBS = FrontierBasedSearch()
    
    rclpy.spin(FBS)

    FBS.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()