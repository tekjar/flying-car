import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid
from scipy.spatial import Voronoi, voronoi_plot_2d
from astar import Astar
import networkx as nx

class VoronoiGraph:
    def __init__(self, map_file):
        data = np.loadtxt(map_file, delimiter=',', dtype='Float64', skiprows=2)

        # altitude, minimum distance to stay away from obstacle
        altitude, safe_distance = 5, 3
        self.grid, self.edges = create_grid(data, altitude, safe_distance)
        self.graph = nx.Graph()

    def grid(self):
        return self.grid

    def create_graph(self):
        for e in self.edges:
            p1 = e[0]
            p2 = e[1]
            dist = np.linalg.norm(np.subtract(p1, p2))
            self.graph.add_edge(p1, p2, weight=dist)

        return self.graph

    #def show(self, x_values, y_values):
    def show(self, x_points, y_points):
        plt.imshow(self.grid, origin='lower', cmap='Greys')
        for e in self.edges:
            p1 = e[0]
            p2 = e[1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

        plt.plot(x_points, y_points, 'r-')

        plt.xlabel('EAST')
        plt.ylabel('NORTH')
        plt.show()

def closest_point(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    closest_point = None
    dist = 100000
    for p in graph.nodes:
        sub = np.subtract(p, current_point)
        d =  np.linalg.norm(sub)
        if d < dist:
            closest_point = p
            dist = d
    return closest_point


start_ne = (25,  100)
goal_ne = (750., 370.)

voronoi = VoronoiGraph('colliders.csv')
graph = voronoi.create_graph()

start_ne = closest_point(graph, start_ne)
goal_ne = closest_point(graph, goal_ne)

print('################')
print(voronoi.edges)
print('################')

astar = Astar(graph)
found, paths = astar.travel(start_ne, goal_ne)

path = astar.trace_back(paths) if found else exit("Couldn't find a path")
xpoints, ypoints = astar.axis_points(path)
voronoi.show(xpoints, ypoints)


