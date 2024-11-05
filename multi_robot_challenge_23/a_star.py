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
    #The A* class is initialized with a fixed row and column length. The grid itself and 
    #the start and goal positions are passed as parameters to the method call  
    def __init__(self, rowlength, columnlength):
        # Define the size of the grid
        self.row = rowlength
        self.column = columnlength

    # Is the cell in the grid?
    def is_valid(self, row, column):
        return (row >= 0) and (row < self.row) and (column >= 0) and (column < self.column)

    # True if there is no obstacle in the cell
    def no_obstacle(self, grid, row, column):
        centre = (grid[row][column] >= 0) and (grid[row][column] <= 90) 

        #TODO: If the cell has an obstacle x cells to the left, right, top, down it is occupied
        if self.is_valid(row + 3, column + 3) and self.is_valid(row - 3, column - 3):
            left = (grid[row][column - 3] >= 0) and (grid[row][column - 3] <= 90)
            right = (grid[row][column + 3] >= 0) and (grid[row][column + 3] <= 90)
            top = (grid[row - 3][column] >= 0) and (grid[row - 3][column] <= 90)
            down = (grid[row + 3][column] >= 0) and (grid[row + 3][column] <= 90)

            return centre and left and right and top and down
        else:
            return False

    # True if the cell is the goal cell
    def is_destination(self, row, column, goal):
        return row == goal[0] and column == goal[1]

    # Heuristic value of cell
    def calculate_h_value(self, row, column, goal):
        return math.pow(math.pow((row - goal[0]), 2) + math.pow((column - goal[1]), 2), 0.5)

        #Manhattan distance. Might be better ??
        #return math.abs(row – goal[0]) + math.abs(column – goal[1])

    #Takes the row and column of the occupancy grid and returns a "real world" point represented by x and y.
    def get_world_pos(self, x, y, map_info):
        x_pos = round(map_info.origin.position.x + y*map_info.resolution, 2)
        y_pos = round(map_info.origin.position.y + x*map_info.resolution, 2)
        world_pos = (x_pos, y_pos)
        return world_pos
    
    #Takes a point and converts it into corresponding row and column in the occupancy grid
    def get_map_coords(self, point, map_info):
        x = int((point[1] - map_info.origin.position.y) / map_info.resolution)
        y = int((point[0] - map_info.origin.position.x) / map_info.resolution)
        return [x, y]


    # Trace the path from source to goal
    def find_path(self, cell_details, goal_x, goal_y, map_info):
        path = []
        while not (cell_details[goal_x][goal_y].parent_i == goal_x and cell_details[goal_x][goal_y].parent_j == goal_y):
            path.append((goal_x, goal_y))
            temp_row = cell_details[goal_x][goal_y].parent_i
            temp_col = cell_details[goal_x][goal_y].parent_j
            goal_x = temp_row
            goal_y = temp_col

        path.append((goal_x, goal_y))

        world_pos_path = []
        for point in path:
            point = self.get_world_pos(point[0], point[1], map_info)
            world_pos_path.append(point)
        world_pos_path.reverse()
        
        reduced_path = world_pos_path[::10]
        if world_pos_path[-1] not in reduced_path:
            reduced_path.append(world_pos_path[-1])

        return reduced_path

    def a_star_search(self, grid, start, goal, map_info):

        start = self.get_map_coords(start, map_info)
        goal = self.get_map_coords(goal, map_info)

        if not self.is_valid(start[0], start[1]) or not self.is_valid(goal[0], goal[1]):
            print("Sourc/goal is invalid")
            return

        if not self.no_obstacle(grid, start[0], start[1]) or not self.no_obstacle(grid, goal[0], goal[1]):
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

            # Check successors for every direction
            directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dir in directions:
                new_i = i + dir[0]
                new_j = j + dir[1]

                if self.is_valid(new_i, new_j) and self.no_obstacle(grid, new_i, new_j) and not closedlist[new_i][new_j]:
                    if self.is_destination(new_i, new_j, goal):
                        # Set the parent of the goal cell
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
                        print("The goal cell is found")
                        # Trace the path back and return it
                        path = self.find_path(cell_details, goal[0], goal[1], map_info)
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


