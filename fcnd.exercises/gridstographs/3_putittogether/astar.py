import numpy as np
from enum import Enum
from queue import PriorityQueue


# breadth-first search is to keep track of visited cells and all your partial plans
# and always expand the shortest partial plan first

# now we add the notion of cost function. which is the total cost of all the actions in a partial plan
# this helps in expansion to optimize the search process for lowest cost
# note that by expanding by bringing next lowest cost node to the front of the queue before analyzing same
# level neighbours, we'll start analyzing next level before analyzing all current level nodes and hence loose
# shortest path

class Action(Enum):
    LEFT = ((0, -1), 1)
    RIGHT = ((0, 1), 1)
    UP = ((-1, 0), 1)
    DOWN = ((1, 0), 1)

    def __str__(self):
        '''
            returns string representation of this action
        '''
        if self == self.LEFT:
            return '>'
        elif self == self.RIGHT:
            return '<'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'

    def move_value(self):
        '''
            returns (row, column) value to add to current position to perform this action
        '''
        return self.value[0]

    def cost(self):
        '''
            returns cost of current action
        '''
        return self.value[1]


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

    def heuristic(self, current_position):
        '''
            Returns manhatten heuristic
        '''

        total_columns = abs(self.goal[1] - current_position[1])
        total_rows = abs(self.goal[0] - current_position[0])

        return total_columns + total_rows

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
        if up_index < 0 or self.map[up_index][current_column] == 1:
            valid.remove(Action.UP)
        # downward movement out of map
        if down_index > max_row_index or self.map[down_index][current_column] == 1:
            valid.remove(Action.DOWN)
        # leftside movement out of map
        if left_index < 0 or self.map[current_row][left_index] == 1:
            valid.remove(Action.LEFT)
        # rightside movement out of map
        if right_index > max_column_index or self.map[current_row][right_index] == 1:
            valid.remove(Action.RIGHT)

        return valid

    def move(self, current, action):
        '''
            moves the current position based on action and returns position after movement
        '''

        drow, dcolumn = action.move_value()
        return current[0] + drow, current[1] + dcolumn

    def travel(self, start, goal):
        self.start = start
        self.goal = goal

        # {currnt_position: (parent, action)}
        paths = {}
        visited = set()
        queue = PriorityQueue()
        found = False

        queue.put((0, start))
        # there are still nodes to traverse through
        while not queue.empty():
            current_total_cost, current_node = queue.get()

            if current_node == goal:
                found = True
                break

            valid_actions = self.valid_actions(current_node)
            for act in valid_actions:
                neighbour = self.move(current_node, act)
                neighbour_cost = act.cost()
                heuristic_cost = self.heuristic(current_node)
                total_cost = current_total_cost + neighbour_cost + heuristic_cost
                # print('current = ', current, ' action = ', action, ' after action = ', neighbour)

                if neighbour not in visited:
                    visited.add(neighbour)
                    queue.put((total_cost, neighbour))
                    paths[neighbour] = (current_node, act)

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
            sgrid[pos[0], pos[1]] = str(action)
            pos = self.move(pos, action)

        sgrid[pos[0], pos[1]] = 'G'
        sgrid[self.start[0], self.start[1]] = 'S'

        print(sgrid)