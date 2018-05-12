from udacidrone.frame_utils import global_to_local, local_to_global

from enum import Enum
from queue import PriorityQueue
import numpy as np
import matplotlib.pyplot as plt
import math

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTHEAST = (-1, 1, math.sqrt(2))
    NORTHWEST = (-1, -1, math.sqrt(2))
    SOUTHEAST = (1, 1, math.sqrt(2))
    SOUTHWEST = (1, -1, math.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle
    # NOTE: In a perfect grid diagonal movements can also be removed when straigt movements are invalid
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTHEAST)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTHEAST)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)
    if x + 1 > n or y - 1 < m or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)

    return valid_actions


def a_star(grid, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
        #exit(1)
    return path[::-1], path_cost


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def plot_plan(grid, start, goal, path):
    plt.imshow(grid, cmap='Greys', origin='lower')
    plt.ylabel('NORTH')
    plt.xlabel('EAST')

    plt.plot(start[1], start[0], marker='x', markersize=5, color="red")
    plt.plot(goal[1], goal[0], marker='x', markersize=5, color="red")

    for p in path:
        plt.plot(p[1], p[0], marker='o', markersize=5, color="blue")

    path_pairs = zip(path[:-1], path[1:])
    for (n1, n2) in path_pairs:
        plt.plot([n1[1], n2[1]], [n1[0], n2[0]], 'green')

    plt.show()

def collinearity_check(p1, p2, p3, epsilon=1e-6):
    points = np.array([[p1[0], p1[1], 1.], [p2[0], p2[1], 1.], [p3[0], p3[1], 1.]])
    det = np.linalg.det(points)
    return abs(det) < epsilon

def prune_path(path):
    pruned_path = [p for p in path]

    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i + 1]
        p3 = pruned_path[i + 2]

        # remove the middle point if colinearity succeeds or else
        # move to next point
        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    return pruned_path

if __name__ == '__main__':
    TARGET_ALTITUDE = 5
    SAFETY_DISTANCE = 5

    grid_start = (82, 86)   # (north, east)
    grid_goal = (451, 395)

    home_longitude = -122.397450
    home_latitude  = 37.792480

    start_longitude = -122.401527
    start_latitude = 37.790394

    goal_longitude = -122.397450
    goal_latitude  = 37.792480

    # (north, east, altitude)
    start_position = global_to_local((start_longitude, start_latitude, 0), (home_longitude, home_latitude, 0))
    goal_position  = global_to_local((goal_longitude, goal_latitude, 0), (home_longitude, home_latitude, 0))

    print('start = {}, goal = {}'.format(start_position, goal_position))

    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
    grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

    print('north offset = {}, east offset = {}'.format(north_offset, east_offset))

    grid_start = (int(start_position[0] - north_offset), int(start_position[1] - east_offset))
    grid_goal  = (int(goal_position[0] - north_offset), int(goal_position[1] - east_offset))

    print('grid start = {}, grid goal = {}'.format(grid_start, grid_goal))

    path, _ = a_star(grid, heuristic, grid_start, grid_goal)
    path = prune_path(path)
    print(path)

    plot_plan(grid, grid_start, grid_goal, path)