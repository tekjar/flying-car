import numpy as np
from enum import Enum
from queue import Queue

# breadth-first search is to keep track of visited cells and all your partial plans
# and always expand the shortest partial plan first

class Action(Enum):
    LEFT = (0, -1)
    RIGHT = (0, 1)
    UP = (-1, 0)
    DOWN = (1, 0)

    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'

class Traveller:
    def __init__(self, map):
        self.map = map
        self.start = None
        self.goal = None

    def map_matrix_shape(self):
        '''
            returns ROWS_MAX_INDEX X COLUMNS_MAX_INDEX
        '''
        return (self.map.shape[0] - 1, self.map.shape[1] - 1)

    def valid_actions(self, current_position):
        current_row, current_column = current_position[0], current_position[1]

        up_index = current_row - 1
        down_index = current_row + 1
        left_index = current_column - 1
        right_index = current_column + 1

        max_row_index, max_column_index = self.map_matrix_shape()

        valid = [Action.UP, Action.DOWN, Action.LEFT, Action.RIGHT]
        
        # print('row = ', current_row, 'column = ', current_column)
        # upward movement out of map
        if up_index < 0 or minimap[up_index][current_column] == 1:
            valid.remove(Action.UP)
        # downward movement out of map
        if down_index > max_row_index or minimap[down_index][current_column] == 1:
            valid.remove(Action.DOWN)
        # leftside movement out of map
        if left_index < 0 or minimap[current_row][left_index] == 1:
            valid.remove(Action.LEFT)
        # rightside movement out of map
        if right_index > max_column_index or minimap[current_row][right_index] == 1:
            valid.remove(Action.RIGHT)

        return valid

    def travel(self, start, goal):
        self.start = start
        self.goal = goal

        # {currnt_position: (parent, action)}
        paths = {}
        visited = set()
        queue = Queue()
        found = False
        
        queue.put(start)
        # there are still nodes to traverse through
        while not queue.empty():
            current = queue.get()

            if current == goal:
                found = True
                break

            valid_actions = self.valid_actions(current)
            for act in valid_actions:
                action = act.value
                neighbour = current[0] + action[0], current[1] + action[1]
                
                #print('current = ', current, ' action = ', action, ' after action = ', neighbour)

                if neighbour not in visited:
                    visited.add(neighbour)
                    queue.put(neighbour)
                    paths[neighbour] = (current, act)

        return found, paths

    def trace_back(self, paths):
        path = []
        
        # trace back from goal
        next = self.goal
        while next != self.start:
            next, action = paths[next]
            path.append(action)

        path = path[::-1]

        return path

    # Define a function to visualize the path
    def visualize(self, path):
        """
        Given a grid, path and start position
        return visual of the path to the goal.

        'S' -> start 
        'G' -> goal
        'O' -> obstacle
        ' ' -> empty
        """
        # Define a grid of string characters for visualization
        sgrid = np.zeros(np.shape(self.map), dtype=np.str)
        sgrid[:] = ' '
        sgrid[self.map[:] == 1] = 'O'

        pos = self.start
        # Fill in the string grid
        for action in path:
            #print('pos = ', pos, ' action = ', action)
            da = action.value
            sgrid[pos[0], pos[1]] = str(action)
            pos = (pos[0] + da[0], pos[1] + da[1])

        sgrid[pos[0], pos[1]] = 'G'
        sgrid[self.start[0], self.start[1]] = 'S'  
        
        print(sgrid)

minimap = np.array([
    [0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 0, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0],
])

traveller = Traveller(minimap)
found, paths = traveller.travel((0,0), (4, 4))

path = traveller.trace_back(paths) if found else exit('No path found')

traveller.visualize(path)
