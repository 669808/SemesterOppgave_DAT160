import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from interfaces.srv import FrontierRequest
from collections import deque
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.colors as mcolors
import time
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import math

class FrontierBasedSearch(Node):
    def __init__(self):
        super().__init__('frontier_based_search')

        self.map_data = None
        self.map_info = None
        self.map = None
        self.map_recieved = False
        self.frontier_centroids = []

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                                depth=5,)

        self.map_subscription = self.create_subscription(
            OccupancyGrid, 
            '/filtered_map', 
            callback=self.clbk_map, 
            qos_profile=qos_profile
        )


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
            QoSProfile(depth=5)
        )

        self.create_subscription(
            LaserScan,
            'tb3_1/scan',
            self.lidar_callback_tb3_1,
            QoSProfile(depth=5)
        )

        time_interval = 0.2
        self.create_timer(time_interval, self.proccess_lidar_readings)

        self.fbs_service = self.create_service (
            FrontierRequest, 
            'frontier_based_search', 
            self.handle_service
        )

        self.pub_frontier_map = self.create_publisher(
            OccupancyGrid, 
            'frontier_map', 
            qos_profile=qos_profile
        )
        
        self.get_logger().info('Service is running')

    def handle_service(self, request, response):
        self.get_logger().info('Service request received')
        robot_position = self.get_map_coords((request.current_pos.x, request.current_pos.y))
        response.success = self.update_frontier_map(robot_position)
        self.get_logger().info(f'Success: {response.success}')

        if not response.success:
            return response

        frontier = self.find_nearest_frontier(robot_position)

        if frontier is None:
            point = Point()
            point.x, point.y = 0.0, 0.0
            response.frontier = point
        else:
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
        frontier_occupancygrid = msg
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

        for i in range(len(frontier_occupancygrid.data)):
            if frontier_occupancygrid.data[i] == 0:
                frontier_occupancygrid.data[i] = -1

        self.frontier_occupancygrid = frontier_occupancygrid

        self.map_data = grid
        self.map_info = msg.info
        self.map_recieved = True
        self.get_logger().info("Map data received")

        # We don't need the subscription anymore
        self.map_subscription.destroy()
  
    
    def update_frontier_map(self, robot_current_pos):
        if self.map_data is None:
            self.get_logger().warn("Map data is not initialized.")
            return False
        
        frontier_cells = self.find_frontier_cells(robot_current_pos)

        if not frontier_cells:
            self.get_logger().warn("No frontier cells found.")
            return False

        centroids = self.find_centroids(frontier_cells)

        self.addValidCentroids(centroids)
        
        self.get_logger().info("Centroids updated")
        self.get_logger().info(f"Found {len(self.frontier_centroids)} centroids")


        if(not self.frontier_centroids):
            self.get_logger().warn("No valid centroids found.")
            return False

        return True
    
    def addValidCentroids(self, centroids):
        distance_to_wall_threshold_metres = 0.3
        existing_centroid_distance_threshold_metres = 1 

        centroid_threshold = int(existing_centroid_distance_threshold_metres / self.map_info.resolution)
        wall_threshold = int(distance_to_wall_threshold_metres / self.map_info.resolution)

        for centroid in centroids:
            if self.tooCloseToWall(centroid, wall_threshold):
                continue

            if self.tooCloseToExistingCentroid(centroid, centroid_threshold):
                continue

            self.frontier_centroids.append(centroid)

    def tooCloseToWall(self, centroid, threshold):
        x, y = centroid
        threshold_cells = [
            (x - threshold, y), (x + threshold, y),
            (x, y - threshold), (x, y + threshold),
            (x - threshold, y - threshold), (x - threshold, y + threshold),
            (x + threshold, y - threshold), (x + threshold, y + threshold)
        ]

        for a, b in threshold_cells:
            intermediate_points = self.breseham_line(x, y, a, b)
            if any(self.isOccupied(x, y) for x, y in intermediate_points):
                return True

        return False

    #NOT TESTED
    def tooCloseToExistingCentroid(self, centroid, threshold):
        x, y = centroid
        for cx, cy in self.frontier_centroids:
            if abs(x - cx) < threshold and abs(y - cy) < threshold:
                return True
        return False

    # Utilize a BFS approach to find the frontier cells from the robots position
    def find_frontier_cells(self, robot_current_pos):
        queue = deque([robot_current_pos])
        visited = set([robot_current_pos])
        frontier_cells = []

        while queue:
            x, y = queue.popleft()

            for nx, ny in self.get_neighbors(x, y):
                if (nx, ny) in visited:
                    continue

                visited.add((nx, ny))

                if self.isOpenSpace(nx, ny):
                    queue.append((nx, ny))
                elif self.isUnexplored(nx, ny):
                    frontier_cells = self.add_cell_ifValid(nx, ny, frontier_cells)

        self.get_logger().info(f"Found {len(frontier_cells)} frontier cells")
        return frontier_cells

    def add_cell_ifValid(self, x, y, frontier_cells: list):
        neighbors = self.get_neighbors(x, y)
        if any(self.isOpenSpace(nx,ny) for nx, ny in neighbors):
                frontier_cells.append((x, y))
        return frontier_cells

    def find_centroids(self, frontier_list: list):
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
                    if (nx, ny) not in visited and (nx, ny) in frontier_list and self.map_data[nx, ny] == -1:
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
                    centroids.append((int(cx), int(cy)))
        return centroids

    def find_nearest_frontier(self, robot_pos):
        if len(self.frontier_centroids) == 0:
            self.get_logger().info("No frontiers found")
            return None

        #Use a BFS approach to find the nearest frontier
        queue = deque([robot_pos])
        visited = set()
        visited.add(robot_pos)
        self.get_logger().info("Finding nearest frontier")
        while queue:
            current_pos = queue.popleft()
            if current_pos in self.frontier_centroids:
                self.get_logger().info("Found nearest frontier")
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
        angles_radians = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)

        # Update the grid for each lidar point
        for angle_rad, distance_metres in zip(angles_radians, ranges):
            if distance_metres < msg.range_min or distance_metres > msg.range_max:
                distance_metres = msg.range_max

            distance = distance_metres / self.map_info.resolution

            x = distance * math.cos(angle_rad)
            y = distance * math.sin(angle_rad)

            grid_x = int(robot_x + x)
            grid_y = int(robot_y + y)

            if 0 <= grid_x < self.map_info.width and 0 <= grid_y < self.map_info.height:
                self.fill_open_cells(robot_x, robot_y, grid_x, grid_y)
        self.pub_frontier_map.publish(self.frontier_occupancygrid)
    
    def fill_open_cells(self, x0, y0, x1, y1):

        intermediate_points  = self.breseham_line(x0, y0, x1, y1)

        for x, y in intermediate_points:
            if self.isOccupied(x, y):
                break
            if self.isUnexplored(x, y):
                self.fill_cells(x, y)

    def fill_cells(self, x, y):
        if self.isUnexplored(x, y):
            self.map_data[x, y] = 0
            map_iter = self.get_map_iter(x, y)
            self.frontier_occupancygrid.data[map_iter] = 0
        neighbors = self.get_neighbors(x, y)
        for nx, ny in neighbors:
            if self.isUnexplored(nx, ny):
                self.map_data[nx, ny] = 0
                map_iter = self.get_map_iter(nx, ny)
                self.frontier_occupancygrid.data[map_iter] = 0

    #Bresenham's line algorithm. Returns a list of points between two points
    def breseham_line(self, x0, y0, x1, y1):
        points = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while x0 != x1 or y0 != y1:
            if self.isValidCell(x0, y0):
                points.append((x0, y0))
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points
    
    
    #Helper functions below

    def isOccupied(self, x, y):
        return self.map_data[x, y] == 100
    
    def isOpenSpace(self, x, y):
        return self.map_data[x, y] == 0
    
    def isUnexplored(self, x, y):
        return self.map_data[x, y] == -1
    
    def isValidCell(self, x, y):
        return 0 <= x < self.map_info.width and 0 <= y < self.map_info.height

    def get_neighbors (self, x, y):
        neighbors = [
            (x - 1, y), (x + 1, y), 
            (x, y - 1), (x, y + 1),
            (x - 1, y - 1), (x - 1, y + 1),
            (x + 1, y - 1), (x + 1, y + 1)
        ]
        return [(nx, ny) for nx, ny in neighbors if 0 <= nx < len(self.map_data) and 0 <= ny < len(self.map_data[0])]

    def get_map_pos(self, map_iter):
        x = int(map_iter/self.map_msg.info.width)
        y = int(map_iter - x*self.map_msg.info.width)
        return [x, y]
    
    def get_map_iter(self, x, y):
        map_iter = x*self.frontier_occupancygrid.info.width + y
        return map_iter
    
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