"""
The Follow Wall algorithm is one of the basic maze navigation methods. It is not very "smart" in that the robot follows
the wall of the maze. It usually takes a long time for the robot to find the goal if at all it succeeds. 
It is an easy to implement method especially in physical robot competitions and a very good beginner's project as simple sensors
can be implemented.
"""

import random
from maze_management import dir_reverse, dir_lt, dir_rt, map_heading_value, Dir, dir_sensors


class FollowWall(object):
    OPEN_WALLS = True # not used here, just for completion
    def __init__(self, maze_map, curr_pos, curr_heading):
        self.maze_map = maze_map
        self.dim = maze_map.dim
        self.RUNZERO = True

        self.x, self.y = curr_pos

        self.curr_heading = map_heading_value[curr_heading]

        self.destination = "CENTER"
        self.at_center = False
        self.maze_explored = False

        self.goal_cells = [[self.dim / 2 - 1, self.dim / 2 - 1], [self.dim / 2 - 1, self.dim / 2],
                           [self.dim / 2, self.dim / 2], [self.dim / 2, self.dim / 2 - 1]]

        self.run_count = 0
        self.row, self.col = self.dim, self.dim

        self.rotation, self.movement = 0, 0
        self.follow_wall = random.choice([Dir.Left, Dir.Right])

    def reset_search(self):
        if self.x is 0 and self.y is 0:
            location = [0, 0]
            walls = [Dir.Down.value]
            self.maze_map.mark_cell_wall(location, Dir.Down, 0)
            self.follow_wall = random.choice([Dir.Left, Dir.Right])
        return False

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
        if next_dir is dir_reverse[self.curr_heading]:
            self.rotation = 90
            self.movement = 0
            self.curr_heading = dir_rt[self.curr_heading]
        else:
            if next_dir is self.curr_heading:
                self.rotation = 0
                self.movement = 1
            elif next_dir is dir_rt[self.curr_heading]:
                self.rotation = 90
                self.movement = 1
            elif next_dir is dir_lt[self.curr_heading]:
                self.rotation = -90
                self.movement = 1

            next_cell = self.maze_map.adjacent_cell([self.x, self.y], next_dir)

            self.x, self.y = next_cell
            self.curr_heading = next_dir
        self.run_count += 1
        return self.rotation, self.movement

    def at_goal(self):
# reached center or home
        if (self.destination is "CENTER" and ([self.x, self.y] in self.goal_cells)) or \
            (self.destination is "HOME" and self.x is 0 and self.y is 0):
            return True
        else:
            return False

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

    def visited_cell(self):
        return self.maze_map.visited_cell([self.x, self.y])

    def next_direction(self, sensors):
        turnright = dir_rt[self.curr_heading]
        turnleft = dir_lt[self.curr_heading]

        cell_q = []
        for i, s_distance in enumerate(sensors):
            if s_distance > 0:
                open_wall = dir_sensors[self.curr_heading][i]
                cell_q.append(open_wall)

        if len(cell_q) is 0:
            return dir_reverse[self.curr_heading]  # go reverse

        if self.follow_wall is Dir.Right:
            if turnright in cell_q:
                return turnright

        if self.follow_wall is Dir.Left:
            if turnleft in cell_q:
                return turnleft

        if self.curr_heading in cell_q:
            return self.curr_heading
        else:
            if self.follow_wall is Dir.Right:
                return turnleft
            else:
                return turnright
