import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid
from skimage.morphology import medial_axis
from skimage.util import invert
from astar import Astar


class Medial:
    def __init__(self, map_file):
        data = np.loadtxt(map_file, delimiter=',', dtype='Float64', skiprows=2)

        # altitude, minimum distance to stay away from obstacle
        altitude, safe_distance = 5, 3
        self.grid = create_grid(data, altitude, safe_distance)

    def get_grid(self):
        return self.grid

    def show(self, x_values, y_values):
        skeleton = medial_axis(invert(self.grid))
        plt.imshow(self.grid, origin='lower')
        plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)
        plt.xlabel('EAST')
        plt.ylabel('NORTH')

        plt.plot(x_values, y_values, 'g')
        plt.show()


start_ne = (25,  100)
goal_ne = (750., 370.)

medial = Medial('colliders.csv')
grid = medial.get_grid()

astar = Astar(grid)
found, paths = astar.travel(start_ne, goal_ne)

path = astar.trace_back(paths) if found else exit("Couldn't find a path")
xpoints, ypoints = astar.axis_points(path)

medial.show(xpoints, ypoints)
