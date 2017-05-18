# Uses Berkley Pacman functions http://ai.berkeley.edu/project_overview.html , http://ai.berkeley.edu/search.html,
# implemented using util.py   https://s3-us-west-2.amazonaws.com/cs188websitecontent/projects/release/search/v1/001/docs/util.html
#
# util.py
# -------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).
"""
Mazes can be navigated if they are considered to be graphs. Each cell then becomes a node in the graph.
Many techniques convert mazes graphs and then solve for the shortest path to goal.
Here there are 4 methods that are implemented - Breadth First, Depth First, Uniform Cost Search and A* (or A Star) search
Of these A* is considered to be the best and uses a heuristic in the calculation of distances.
 
 This implementation is inspired by the UC Berkley PACMAN project and the algorithms that were used. 
"""

from maze_management import dir_reverse, dir_lt, dir_rt, map_heading_value, Dir, dir_sensors
from util import *


class searchAgents(object):
    OPEN_WALLS = False

    def __init__(self, maze_map, curr_pos, curr_heading):

        self.maze_map = maze_map
        self.dim = maze_map.dim
        self.RUNZERO = True
        self.run_count = 0
        self.x, self.y = curr_pos
        #self.y = curr_pos[1]

        self.curr_heading = map_heading_value[curr_heading]

        self.destination = "CENTER"
        self.at_center = False
        self.maze_explored = False
        self.goalLoc = [0, 0]
        self.goal_cells = [[self.dim / 2 - 1, self.dim / 2 - 1], [self.dim / 2 - 1, self.dim / 2],
                           [self.dim / 2, self.dim / 2], [self.dim / 2, self.dim / 2 - 1]]
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
            if self.destination is "CENTER":
                self.at_center = True
                self.goalLoc = [self.x, self.y]
                self.maze_map.goal_cell_id([self.x, self.y])

        if self.reset_search(): return self.reset_run()

        next_dir = self.next_direction(sensors)

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

            self.x, self.y = self.maze_map.adjacent_cell([self.x, self.y], next_dir)

            self.curr_heading = next_dir

        self.run_count += 1
        return self.rotation, self.movement

    def at_goal(self):
        if (self.destination is "CENTER" and ([self.x, self.y] in self.goal_cells)) or\
                (self.destination is "HOME" and self.x is 0 and self.y is 0):
            return True
        else:
            return False

    def visited_cell(self):
        return self.maze_map.visited_cell([self.x, self.y])

    def mark_cell_visited(self):
        return self.maze_map.mark_cell_visited([self.x, self.y])

    def reset_search(self):
        maze_searched_percent = (sum(map(sum, self.maze_map.explored)) / float(self.dim ** 2)) * 100
        if maze_searched_percent >= 85.0:
            return self.at_center and maze_searched_percent

    def check_if_goal(self, curr_pos):

        if curr_pos in self.goal_cells:
            return True
        else:
            return False

    def reset_run(self):

        if self.RUNZERO:
            self.before_reset()
            self.RUNZERO = False
            self.x, self.y = 0, 0

            self.curr_heading = Dir.Up
            self.rotation = 90
            self.destination = "CENTER"
            self.current_path = [([self.x, self.y], self.rotation, self.curr_heading.value)]
            self.maze_explored = True
            self.at_center = False
            print (" Phase 1 complete -  Step count ", self.run_count)

            self.run_count = 0
        return 'Reset', 'Reset'

    def next_direction(self, sensors):

        goal_q = []
        new_cell_q = []
        cell_q = []

        for i, s_distance in enumerate(sensors):
            if s_distance > 0:
                open_wall = dir_sensors[self.curr_heading][i]

                next_to = self.maze_map.adjacent_cell([self.x, self.y], open_wall)
                if self.at_center and self.check_if_goal(next_to):
                    continue

                cell_q.append(open_wall)

                if self.check_if_goal(next_to):
                    goal_q.append(open_wall)

                visited_cell = self.maze_map.explored[next_to[1]][next_to[0]]
                if not visited_cell:
                    new_cell_q.append(open_wall)

        if goal_q:
            cell_dir_next = random.choice(goal_q)
        elif new_cell_q:
            cell_dir_next = random.choice(new_cell_q)
        elif cell_q:
            cell_dir_next = random.choice(cell_q)
        else:
            cell_dir_next = dir_reverse[self.curr_heading]  # go reverse

        return cell_dir_next

    ####
    @staticmethod
    def search_agent(problem, frontier):

        explored = []
        frontier.push([(problem.getStartState(), Dir.Up, 0)])

        while not frontier.isEmpty():
            path = frontier.pop()

            s = path[-1]
            s = s[0]
            if problem.isGoalState(s):
                return [x[1] for x in path][1:]

            if s not in explored:
                explored.append(s)

                for successor in problem.getSuccessors(s):
                    if successor[0] not in explored:
                        successorPath = path[:]
                        successorPath.append(successor)
                        frontier.push(successorPath)

        return []


class PositionSearchProblem:
    """
     A search problem defines the state space, start state, goal test, successor
     function and cost function.  This search problem can be used to find paths
     to a particular point on the pacman board.

     The state space consists of (x,y) positions in a pacman game.

     Note: this search problem is fully specified; you should NOT change it.
     from UC Nerkley PACMAN project https://s3-us-west-2.amazonaws.com/cs188websitecontent/projects/release/search/v1/001/docs/searchAgents.html
    """

    def __init__(self, maze_map, start=[0, 0], goal=[256, 256], costFn=lambda x: 1):

        self.maze_map = maze_map
        self.start = start
        self.goal = goal
        self.costFn = costFn

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        isGoal = state is self.goal
        return isGoal

    def getSuccessors(self, state):

        successors = []
        for direction in list(Dir):
            x, y = state
            if not self.maze_map.wall_hit([x, y], direction):
                nextState = self.maze_map.adjacent_cell([x, y], direction)
                cost = self.costFn(nextState)
                successors.append((nextState, direction, cost))

        return successors

    def getCostOfActions(self, directions):
        # Returns the cost of a particular sequence of actions.  If those actions
        # include an illegal move, return 999999
        if directions is None: return 999999
        x, y = self.getStartState()
        cost = 0
        for direction in directions:
            if self.maze_map.wall_hit([x, y], direction): return 999999
            cost += self.costFn((x, y))
        return cost

#    The individual search methods are BreadthFirst, DepthFirst, UniformCost and A* (ASTAR)
#
class BreadthFirst(searchAgents):
    def before_reset(self):
        problem = PositionSearchProblem(self.maze_map, goal=self.goalLoc)
        frontier = PriorityQueueWithFunction(len)
        self.searched_path = self.search_agent(problem, frontier)


class DepthFirst(searchAgents):
    def before_reset(self):
        problem = PositionSearchProblem(self.maze_map, goal=self.goalLoc)
        frontier = Stack()
        self.searched_path = self.search_agent(problem, frontier)


class UniformCost(searchAgents):
    def before_reset(self):
        problem = PositionSearchProblem(self.maze_map, goal=self.goalLoc)
        cost = lambda aPath: problem.getCostOfActions([x[1] for x in aPath])
        frontier = PriorityQueueWithFunction(cost)
        self.searched_path = self.search_agent(problem, frontier)


class Astar(searchAgents):
    def before_reset(self):
        problem = PositionSearchProblem(self.maze_map, goal=self.goalLoc)

        cost = lambda aPath: problem.getCostOfActions([x[1] for x in aPath]) + self.manhattanHeuristic(
            aPath[len(aPath) - 1][0], problem)
        frontier = PriorityQueueWithFunction(cost)

        self.searched_path = self.search_agent(problem, frontier)

    @staticmethod
    def manhattanHeuristic(position, problem):
        """The Manhattan distance heuristic for a PositionSearchProblem"""
# calculates the distance between the goal and current position
        xy1 = position
        xy2 = problem.goal
        return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])
