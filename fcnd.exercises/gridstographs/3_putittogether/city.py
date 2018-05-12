import numpy as np
import matplotlib.pyplot as plt
from astar import Traveller
from grid import create_grid


def move(current, action):
    '''
        moves the current position based on action and returns position after movement
    '''

    drow, dcolumn = action.move_value()
    return current[0] + drow, current[1] + dcolumn

plt.rcParams['figure.figsize'] = 12, 12

data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
print(data)

altitude = 5
# minimum distance to stay away from obstacle
safe_distance = 3

grid = create_grid(data, altitude, safe_distance)
print(grid)

# plt.imshow(grid, origin='lower')
# plt.xlabel('EAST')
# plt.ylabel('NORTH')
# plt.show()

start_ne = (25,  100)
goal_ne = (750., 370.)

traveller = Traveller(grid)
found, paths = traveller.travel(start_ne, goal_ne)

print('found = ', found)

path = traveller.trace_back(paths) if found else exit('No path found')

if found:
    print('path len = ', len(path))

plt.imshow(grid, cmap='Greys', origin='lower')

# For the purposes of the visual the east coordinate lay along
# the x-axis and the north coordinates long the y-axis.
plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')

sgrid_row = []
sgrid_column = []
pos = start_ne

# Fill in the string grid
for action in path:
    sgrid_row.append(pos[0])
    sgrid_column.append(pos[1])
    pos = move(pos, action)

# pp = np.array(path)
plt.plot(sgrid_column, sgrid_row, 'g')
#
plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()