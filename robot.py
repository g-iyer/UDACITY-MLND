from maze_management import MazeObject
from follow_wall import FollowWall
from flood_fill import FloodFill
from graph_search import *

'''
This Robot.py implementation uses a options.txt file to specify which search method to use 
The options.txt file has one single line and one value at a time. 
The valid values are given below:
                 To run this method             Use
                 ------------------             --
                 Follow Wall                    WF
    Graph search Breadth First                  BF
                 Depth First                    DF
                 Uniform Cost                   UC
                 A Star (A*)                    ASTAR
                   
                 Flood Fill                     FF
'''
SEARCH_ALGOS = {
    'WF': FollowWall,
    'DF': DepthFirst,
    'BF': BreadthFirst,
    'UC': UniformCost,
    'ASTAR': Astar,
    'FF': FloodFill
}


class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
        algorithm = open('options.txt', 'r').read()
        print ('options ', algorithm)
        location = [0, 0]
        curr_heading = 'up'

        try:
            SearchClass = SEARCH_ALGOS[algorithm]
        except KeyError:
            raise Exception('\n\n%s is not a valid search algorithm. Please select one of %s\n\n' % (
            algorithm, SEARCH_ALGOS.keys()))
        maze_map = MazeObject(maze_dim, SearchClass.OPEN_WALLS)
        self.algo = SearchClass(maze_map, location, curr_heading)

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        rotation, movement = self.algo.explore_maze(sensors)

        return rotation, movement
