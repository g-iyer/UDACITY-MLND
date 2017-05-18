"""
Flood Fill algorithm is used extensively in paint and similar image manipulation programs. 
It is typically used to fill colors in a maze representing an image. 
"""

from maze_management import dir_reverse, dir_lt, dir_rt, map_heading_value, Dir


class FloodFill(object):
    OPEN_WALLS = True

    def __init__(self, maze_map, curr_pos, curr_heading):
        self.maze_map = maze_map
        self.dim = maze_map.dim

        self.RUNZERO = True

        self.x, self.y  = curr_pos

        self.curr_heading = map_heading_value[curr_heading]

        self.destination = "CENTER"  # HOME,CENTER
        self.at_center = False
        self.maze_explored = False
        self.destinationLoc = [0, 0]
        self.goal_cells = [[self.dim / 2 - 1, self.dim / 2 - 1], [self.dim / 2 - 1, self.dim / 2],
                           [self.dim / 2, self.dim / 2], [self.dim / 2, self.dim / 2 - 1]]
        self.home_cell = [0, 0]
        # movement counters
        self.run_count = 0
        self.at_center_counter = 0
        self.at_start_counter = 0

        self.row, self.col = self.dim, self.dim

        self.init_cost = self.dim ** 2
        self.depth = [[self.init_cost for _ in range(self.row)] for _ in range(self.col)]
        self.rotation, self.movement = 0, 0

    def explore_maze(self, sensors):
        if self.RUNZERO:
            print("Exploring run - count = {}".format(self.run_count))
        else:
            print("Final run - count = {}".format(self.run_count))

        if not self.visited_cell():
            self.maze_map.mark_maze_walls([self.x, self.y], self.curr_heading, sensors)
            self.maze_map.mark_cell_visited([self.x, self.y])

        if self.at_goal():
            self.goal_found()

        if self.reset_search():
            return self.reset_run()

        next_dir = self.next_direction(sensors)
# 90 is CW rotation and -90 is CCW rotation
        if next_dir is dir_reverse[self.curr_heading]:  # turn back
            self.rotation = 90
            self.movement = 0
            self.curr_heading = dir_rt[self.curr_heading]
        else:
            if next_dir is self.curr_heading: # no block, keep moving in the same direction
                self.rotation = 0
                self.movement = 1
            elif next_dir is dir_rt[self.curr_heading]: # turn right
                self.rotation = 90
                self.movement = 1
            elif next_dir is dir_lt[self.curr_heading]: # turn left
                self.rotation = -90
                self.movement = 1

            self.x, self.y = self.maze_map.adjacent_cell([self.x, self.y], next_dir)
            self.curr_heading = next_dir

            self.run_count += 1
        return self.rotation, self.movement

    def reset_run(self):

        if self.RUNZERO:
            self.RUNZERO = False
            self.x = 0
            self.y = 0
            self.curr_heading = Dir.Up
            self.rotation = 90
            self.destination = "CENTER"
            self.current_path = [([self.x, self.y], self.rotation, self.curr_heading.value)]
            self.maze_explored = True
            self.at_center = False
            self.run_count = 0
            print " Phase 1 complete "

        return 'Reset', 'Reset'

    def at_goal(self):
# reached center or home
        if (self.destination is "CENTER" and ([self.x, self.y] in self.goal_cells)) or\
                (self.destination is "HOME" and self.x is 0 and self.y is 0):
            return True
        else:
            return False

    def visited_cell(self):
        return self.maze_map.visited_cell([self.x, self.y])

    def set_cell_depth(self, cell, depth):
        x, y = cell
        self.depth[y][x] = depth

    def get_cell_depth(self, cell):
        x, y = cell
        return self.depth[y][x]

    def flood_fill(self):
        queue = []
        g_bottom_left, g_top_left, g_top_right, g_bottom_right = self.goal_cells

        self.depth = [[self.init_cost for _ in range(self.row)] for _ in range(self.col)]

        if self.destination is "HOME":
            start_cell = [0, 0]
            self.set_cell_depth(start_cell, 0)
            queue.append(start_cell)
        else:
            self.set_cell_depth(g_top_left, 0)
            self.set_cell_depth(g_bottom_left, 0)
            self.set_cell_depth(g_top_right, 0)
            self.set_cell_depth(g_bottom_right, 0)
            queue.append(g_top_left)
            queue.append(g_top_right)
            queue.append(g_bottom_right)
            queue.append(g_bottom_left)

        while len(queue) > 0:
            cell = queue.pop(0)
            curr_depth = self.get_cell_depth(cell)

            for direction in list(Dir):
                if not self.maze_map.wall_hit(cell, direction):
                    next_to = self.maze_map.adjacent_cell(cell, direction)
                    if next_to:
                        next_to_depth = self.get_cell_depth(next_to)
                        if (curr_depth + 1) < next_to_depth:
                            queue.append(next_to)
                            self.set_cell_depth(next_to, curr_depth + 1)

    def reset_search(self):
        return self.at_center and self.at_center_counter is self.at_start_counter

    def goal_found(self):

        if self.destination is "CENTER":
            self.at_center = True
            self.maze_map.goal_cell_id([self.x, self.y])
            self.at_center_counter = self.run_count
        else:
            self.at_start_counter = self.run_count

        self.run_count = 0

        if self.at_center_counter is self.at_start_counter:
            return

        self.destination = "HOME" if self.destination is "CENTER" else "CENTER"
        self.flood_fill()

    def next_direction(self, sensors):

        curr_cell = [self.x, self.y]
        cell_depth = self.get_cell_depth(curr_cell)
        cell_dir_next = None

        for direction in list(Dir):
            if not self.maze_map.wall_hit(curr_cell, direction):
                next_to = self.maze_map.adjacent_cell(curr_cell, direction)
                next_to_depth = self.get_cell_depth(next_to) if next_to else FloodFill.init_cost
                if cell_depth > next_to_depth:
                    cell_dir_next = direction
                    cell_depth = next_to_depth

        if cell_dir_next:
            return cell_dir_next
        else:
            self.flood_fill()
            return self.next_direction(sensors)