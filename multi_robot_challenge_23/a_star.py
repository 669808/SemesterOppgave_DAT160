import heapq

class Cell:
    def __init__(self):
        self.parent_i = 0 
        self.parent_j = 0

        self.f = float('inf')
        self.g = float('inf')
        self.h = 0

class AStar:
    def __init__(self, rowlength, columnlength, grid, map_info):
        self.row = rowlength
        self.column = columnlength
        self.grid = grid
        self.map_info = map_info

    def cellIsOpenSpace(self, row, column):
        return self.grid[row][column] == 0

    def cellIsValid(self, row, column):
        return (row >= 0) and (row < self.row) and (column >= 0) and (column < self.column)

    # Note that a cell is considered an obstacle if the cell is too close to a obstacle cell
    def noObstacleInCell(self, row, column):
        distance_from_obstacle_metres = 0.35 # This is found by trial and error
        distance_from_obstacle = int(distance_from_obstacle_metres / self.map_info.resolution)

        if not (self.cellIsValid(row + distance_from_obstacle, column + distance_from_obstacle) 
                and self.cellIsValid(row - distance_from_obstacle, column - distance_from_obstacle)):
            return False

        left = self.cellIsOpenSpace(row, column - distance_from_obstacle)
        right = self.cellIsOpenSpace(row, column + distance_from_obstacle)
        top = self.cellIsOpenSpace(row - distance_from_obstacle, column)
        down = self.cellIsOpenSpace(row + distance_from_obstacle, column)
        centre = self.cellIsOpenSpace(row, column)
        topleft = self.cellIsOpenSpace(row - distance_from_obstacle, column - distance_from_obstacle)
        topright = self.cellIsOpenSpace(row - distance_from_obstacle, column + distance_from_obstacle)
        downleft = self.cellIsOpenSpace(row + distance_from_obstacle, column - distance_from_obstacle)
        downright = self.cellIsOpenSpace(row + distance_from_obstacle, column + distance_from_obstacle)

        bool1 = centre and left and right and top and down
        bool2 = topleft and topright and downleft and downright
        return bool1 and bool2

    def cellIsDestination(self, row, column, goal):
        return row == goal[0] and column == goal[1]

    def calculateHeuristicValue(self, row, column, goal):
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
    def findPath(self, cell_details, goal_x, goal_y):
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
            if i % 2 == 0:
                continue
            point = self.get_world_pos(point[0], point[1])
            world_pos_path.append(point)
        world_pos_path.reverse()
        
        return world_pos_path

    def main_loop(self, closedlist, cell_details, open_list, goal):
        while len(open_list) > 0:
            p = heapq.heappop(open_list)
            i = p[1]
            j = p[2]
            closedlist[i][j] = True

            directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dir in directions:
                new_i = i + dir[0]
                new_j = j + dir[1]

                if not (self.cellIsValid(new_i, new_j) and self.noObstacleInCell(new_i, new_j) and not closedlist[new_i][new_j]):
                    continue

                if self.cellIsDestination(new_i, new_j, goal):
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    print("The goal cell is found")
                    path = self.findPath(cell_details, goal[0], goal[1])
                    return path
                    
                g_new = cell_details[i][j].g + 1.0
                h_new = self.calculateHeuristicValue(new_i, new_j, goal)
                f_new = g_new + h_new

                if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                    heapq.heappush(open_list, (f_new, new_i, new_j))
                    cell_details[new_i][new_j].f = f_new
                    cell_details[new_i][new_j].g = g_new
                    cell_details[new_i][new_j].h = h_new
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j

    def a_star_search(self, start, goal):

        start = self.get_map_coords(start)
        goal = self.get_map_coords(goal)

        if not self.cellIsValid(start[0], start[1]) or not self.cellIsValid(goal[0], goal[1]):
            print("Source/goal is invalid")
            return

        if not self.noObstacleInCell(start[0], start[1]) or not self.noObstacleInCell(goal[0], goal[1]):
            print("Either the start or destination is blocked")
            return

        if self.cellIsDestination(start[0], start[1], goal):
            print("Bot is at the goal")
            return

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

        # Main loop of A* search algorithm
        path = self.main_loop(closedlist, cell_details, open_list, goal)

        if path is not None:
            return path
        
        print("A* algorithm failed. Destination not found")
        return None
    



