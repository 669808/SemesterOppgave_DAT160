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
        boolean = (grid[row][column] >= 0) and (grid[row][column] <= 90) 
        return boolean

    # True if the cell is the goal cell
    def is_destination(self, row, column, goal):
        return row == goal[0] and column == goal[1]

    # Heuristic value of cell
    def calculate_h_value(self, row, column, goal):
        return math.pow(math.pow((row - goal[0]), 2) + math.pow((column - goal[1]), 2), 0.5)

        #Manhattan distance. Might be better ??
        #return math.abs(row – goal[0]) + math.abs(column – goal[1])

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
        path.reverse()
        return path

        
    def a_star_search(self, grid, start, goal):
        # Check if the source and goal are valid
        if not self.is_valid(start[0], start[1]) or not self.is_valid(goal[0], goal[1]):
            print("Source or goal is invalid")
            return

        # Check if the source and goal are unblocked
        if not self.no_obstacle(grid, start[0], start[1]) or not self.no_obstacle(grid, goal[0], goal[1]):
            print("Source or the goal is blocked")
            return

        # Check if we are already at the goal
        if self.is_destination(start[0], start[1], goal):
            print("We are already at the goal")
            return

        # Initialize the closed list (visited cells)
        closedlist = [[False for _ in range(self.column)] for _ in range(self.row)]
        # Initialize the details of each cell
        cell_details = [[Cell() for _ in range(self.column)] for _ in range(self.row)]

        # Initialize the start cell details
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
            # Pop the cell with the smallest f value from the open list
            p = heapq.heappop(open_list)

            # Mark the cell as visited
            i = p[1]
            j = p[2]
            closedlist[i][j] = True

            # For each direction, check the successors
            directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dir in directions:
                new_i = i + dir[0]
                new_j = j + dir[1]

                # If the successor is valid, unblocked, and not visited
                if self.is_valid(new_i, new_j) and self.no_obstacle(grid, new_i, new_j) and not closedlist[new_i][new_j]:
                    # If the successor is the goal
                    if self.is_destination(new_i, new_j, goal):
                        # Set the parent of the goal cell
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
                        print("The goal cell is found")
                        # Trace and print the path from source to goal
                        path = self.find_path(cell_details, goal[0], goal[1])
                        found_dest = True
                        return path
                    else:
                        # Calculate the new f, g, and h values
                        g_new = cell_details[i][j].g + 1.0
                        h_new = self.calculate_h_value(new_i, new_j, goal)
                        f_new = g_new + h_new

                        # If the cell is not in the open list or the new f value is smaller
                        if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                            # Add the cell to the open list
                            heapq.heappush(open_list, (f_new, new_i, new_j))
                            # Update the cell details
                            cell_details[new_i][new_j].f = f_new
                            cell_details[new_i][new_j].g = g_new
                            cell_details[new_i][new_j].h = h_new
                            cell_details[new_i][new_j].parent_i = i
                            cell_details[new_i][new_j].parent_j = j

        if not found_dest:
            print("A* algorithm failed. Destination not found")

def main():
    grid = [
        [1  , 100, 1  , 1  , 1  , 1  , 100, 1  , 1  , 1  ],
        [1  , 1  , 1  , 100, 1  , 1  , 1  , 100, 1  , 1  ],
        [1  , 1  , 1  , 100, 1  , 1  , 100, 1  , 100, 1  ],
        [100, 100, 1  , 100, 1  , 100, 100, 100, 100, 1  ],
        [1  , 1  , 1  , 100, 1  , 1  , 1  , 100, 1  , 100],
        [1  , 100, 1  , 1  , 1  , 1  , 100, 1  , 100, 100],
        [1  , 100, 100, 100, 100, 1  , 100, 100, 100, 1  ],
        [1  , 100, 1  , 1  , 1  , 1  , 100, 1  , 1  , 1  ],
        [1  , 1  , 1  , 100, 100, 100, 1  , 100, 100, 1  ]
    ]

    # Define the source and goal
    start = [8, 0]
    goal = [0, 0]

    # Run the A* search algorithm
    astar = AStar(9, 10)
    path = astar.a_star_search(grid, start, goal)

    for i in path:
        print("->", i, end=" ")
    print()

if __name__ == "__main__":
    main()
