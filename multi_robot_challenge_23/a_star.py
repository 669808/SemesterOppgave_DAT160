import math
import heapq

# Define the Cell class
class Cell:
    def __init__(self):
        # Row and Column
        self.parent_i = 0 
        self.parent_j = 0

        self.f = float('inf')
        self.g = float('inf')
        self.h = 0

class AStar:
    #The A* class is initialized with a fixed row and column length. The self.grid itself and 
    #the start and goal positions are passed as parameters to the method call  
    def __init__(self, rowlength, columnlength, grid, map_info):
        # Define the size of the self.grid
        self.row = rowlength
        self.column = columnlength
        self.grid = grid
        self.map_info = map_info

    # Is the cell in the self.grid?
    def is_valid(self, row, column):
        return (row >= 0) and (row < self.row) and (column >= 0) and (column < self.column)

    # True if there is no obstacle in the cell. A cell is considered an obstacle if there is an 
    # obstacle in the cell itself or an obstacle close to it.
    def no_obstacle(self, row, column):

        distance_from_obstacle = 30 # cm
        #distance_from_obstacle = int(300 / self.map_info.resolution)  # Convert 300 cm to grid cells


        if not (self.is_valid(row + distance_from_obstacle, column + distance_from_obstacle) 
                and self.is_valid(row - distance_from_obstacle, column - distance_from_obstacle)):
            return False

        left = (self.grid[row][column - distance_from_obstacle] >= 0) and (self.grid[row][column - distance_from_obstacle] <= 70)
        right = (self.grid[row][column + distance_from_obstacle] >= 0) and (self.grid[row][column + distance_from_obstacle] <= 70)
        top = (self.grid[row - distance_from_obstacle][column] >= 0) and (self.grid[row - distance_from_obstacle][column] <= 70)
        down = (self.grid[row + distance_from_obstacle][column] >= 0) and (self.grid[row + distance_from_obstacle][column] <= 70)
        centre = (self.grid[row][column] >= 0) and (self.grid[row][column] <= 70) 

        return centre and left and right and top and down

    # True if the cell is the goal cell
    def is_destination(self, row, column, goal):
        return row == goal[0] and column == goal[1]

    # Heuristic value of cell
    def calculate_h_value(self, row, column, goal):
        #Manhattan distance
        return abs(row - goal[0]) + abs(column - goal[1])

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
        return [x, y]


    # Trace the path from source to goal
    def find_path(self, cell_details, goal_x, goal_y):
        path = []
        while not (cell_details[goal_x][goal_y].parent_i == goal_x and cell_details[goal_x][goal_y].parent_j == goal_y):
            path.append((goal_x, goal_y))
            temp_row = cell_details[goal_x][goal_y].parent_i
            temp_col = cell_details[goal_x][goal_y].parent_j
            goal_x = temp_row
            goal_y = temp_col

        path.append((goal_x, goal_y))

        world_pos_path = []
        for i, point in enumerate(path):
            if i % 5 == 0 or i == len(path) - 1:
                point = self.get_world_pos(point[0], point[1])
                world_pos_path.append(point)
        world_pos_path.reverse()
        
        return world_pos_path

    def a_star_search(self, start, goal):

        start = self.get_map_coords(start)
        goal = self.get_map_coords(goal)

        if not self.is_valid(start[0], start[1]) or not self.is_valid(goal[0], goal[1]):
            print("Source/goal is invalid")
            return

        if not self.no_obstacle(start[0], start[1]) or not self.no_obstacle(goal[0], goal[1]):
            print("Either the start or destination is blocked")
            return

        if self.is_destination(start[0], start[1], goal):
            print("Bot is at the goal")
            return

        # Visited cells
        closedlist = [[False for _ in range(self.column)] for _ in range(self.row)]

        cell_details = [[Cell() for _ in range(self.column)] for _ in range(self.row)]

        i = start[0]
        j = start[1]
        cell_details[i][j].f = 0
        cell_details[i][j].g = 0
        cell_details[i][j].h = 0
        cell_details[i][j].parent_i = i
        cell_details[i][j].parent_j = j

        open_list = []
        heapq.heappush(open_list, (0.0, i, j))
        found_dest = False

        # Main loop of A* search algorithm
        while len(open_list) > 0:
            p = heapq.heappop(open_list)
            i = p[1]
            j = p[2]
            closedlist[i][j] = True

            # Check successors for every direction. Since we dont need to check every cm
            # we can skip some cells. The neighbors are therefore 10 cells in each direction.
            directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dir in directions:
                new_i = i + dir[0]
                new_j = j + dir[1]

                if self.is_valid(new_i, new_j) and self.no_obstacle(new_i, new_j) and not closedlist[new_i][new_j]:
                    if self.is_destination(new_i, new_j, goal):
                        # Set the parent of the goal cell
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
                        print("The goal cell is found")
                        # Trace the path back and return it
                        path = self.find_path(cell_details, goal[0], goal[1])
                        found_dest = True
                        return path
                    else:
                        # Calculate f, g, and h
                        g_new = cell_details[i][j].g + 1.0
                        h_new = self.calculate_h_value(new_i, new_j, goal)
                        f_new = g_new + h_new

                        if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                            heapq.heappush(open_list, (f_new, new_i, new_j))
                            cell_details[new_i][new_j].f = f_new
                            cell_details[new_i][new_j].g = g_new
                            cell_details[new_i][new_j].h = h_new
                            cell_details[new_i][new_j].parent_i = i
                            cell_details[new_i][new_j].parent_j = j

        if not found_dest:
            print("A* algorithm failed. Destination not found")


