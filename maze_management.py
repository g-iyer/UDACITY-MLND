"""
maze_management.py
The maze management is implemented here. The maze object is defined and all the relevant directions are mapped.
"""
from enum import Enum
class Dir(Enum):
    #    Enum class of 4 cardinal directions
    Up = 0
    Right = 1
    Down = 2
    Left = 3

# maps for various types of direction lookup
map_heading_value = {'up': Dir.Up, 'right': Dir.Right, 'down': Dir.Down, 'left': Dir.Left}

dir_rt = {Dir.Up: Dir.Right, Dir.Right: Dir.Down, Dir.Down: Dir.Left, Dir.Left: Dir.Up}

dir_lt = {Dir.Up: Dir.Left, Dir.Right: Dir.Up, Dir.Down: Dir.Right, Dir.Left: Dir.Down}

dir_reverse = {Dir.Up: Dir.Down, Dir.Right: Dir.Left, Dir.Down: Dir.Up, Dir.Left: Dir.Right}

dir_sensors = {Dir.Up: [Dir.Left, Dir.Up, Dir.Right], Dir.Right: [Dir.Up, Dir.Right, Dir.Down],
               Dir.Down: [Dir.Right, Dir.Down, Dir.Left], Dir.Left: [Dir.Down, Dir.Left, Dir.Up]}

# Main class definition for the maze object used by all methods
# the maze is defined as a list of values for the walls - open or closed as needed.
# the size of the maze is dim ** 2 or (dim x dim)
#
class MazeObject(object):
    def __init__(self, maze_dim, openwall=True):
        self.dim = maze_dim
        self.openwall = openwall

        self.explored = [[False for _ in range(self.dim)] for _ in range(self.dim)]

        if openwall:
            cell_wall_value = 1
        else:
            cell_wall_value = 0


        self.maze_walls = [[[cell_wall_value] * 4 for _ in range(maze_dim)] for _ in range(maze_dim)]
        self.goal_cells = [[self.dim / 2 - 1, self.dim / 2 - 1], [self.dim / 2 - 1, self.dim / 2],
                           [self.dim / 2, self.dim / 2], [self.dim / 2, self.dim / 2 - 1]]

    def mark_maze_walls(self, curr_pos, curr_heading, sensors):
        walls = []
# check to see if there is opening ahead (indicated by a non-zero value in one or more of the sensors
        for i, s_distance in enumerate(sensors):
            wall = dir_sensors[curr_heading][i]
            if s_distance is 0:
                walls.append(wall.value)

            if self.openwall and s_distance is 0:       # wall present
                self.mark_cell_wall(curr_pos, wall, 0)
            if not self.openwall and s_distance > 0:    # no wall
                self.mark_cell_wall(curr_pos, wall, 1)

    def mark_cell_wall(self, curr_pos, direction, cell_wall_value):
        x, y = curr_pos
        self.maze_walls[y][x][direction.value] = cell_wall_value
        next_to = self.adjacent_cell(curr_pos, direction)
        if next_to:
            x, y = next_to
            if direction is Dir.Up:
                self.maze_walls[y][x][Dir.Down.value] = cell_wall_value
            elif direction is Dir.Right:
                self.maze_walls[y][x][Dir.Left.value] = cell_wall_value
            elif direction is Dir.Down:
                self.maze_walls[y][x][Dir.Up.value] = cell_wall_value
            elif direction is Dir.Left:
                self.maze_walls[y][x][Dir.Right.value] = cell_wall_value

    def wall_hit(self, curr_pos, direction):
        x, y = curr_pos
        if direction is Dir.Up and (y is (self.dim - 1)):
            return True
        elif direction is Dir.Right and (x is (self.dim - 1)):
            return True
        elif direction is Dir.Down and (y is 0):
            return True
        elif direction is Dir.Left and (x is 0):
            return True
        else:
            return self.maze_walls[y][x][direction.value] is 0

    def adjacent_cell(self, curr_pos, direction):
        x, y = curr_pos
        if direction is Dir.Up and (y != (self.dim - 1)):
            return [x, y + 1]
        elif direction is Dir.Right and (x != (self.dim - 1)):
            return [x + 1, y]
        elif direction is Dir.Down and (y != 0):
            return [x, y - 1]
        elif direction is Dir.Left and (x != 0):
            return [x - 1, y]
        else:
            return None

    def goal_cell_id(self, curr_cell):

        g_bottom_left, g_top_left, g_top_right, g_bottom_right = self.goal_cells

        if curr_cell != g_top_left:
            self.mark_cell_wall(g_top_left, Dir.Up, 0)
            self.mark_cell_wall(g_top_left, Dir.Left, 0)

        if curr_cell != g_top_right:
            self.mark_cell_wall(g_top_right, Dir.Up, 0)
            self.mark_cell_wall(g_top_right, Dir.Right, 0)

        if curr_cell != g_bottom_right:
            self.mark_cell_wall(g_bottom_right, Dir.Right, 0)
            self.mark_cell_wall(g_bottom_right, Dir.Down, 0)

        if curr_cell != g_bottom_left:
            self.mark_cell_wall(g_bottom_left, Dir.Down, 0)
            self.mark_cell_wall(g_bottom_left, Dir.Left, 0)

    def visited_cell(self, curr_pos):
        x, y = curr_pos
        return self.explored[y][x]

    def mark_cell_visited(self, curr_pos):
        x, y = curr_pos
        self.explored[y][x] = True
