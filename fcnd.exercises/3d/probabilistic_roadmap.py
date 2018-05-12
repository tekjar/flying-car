from sklearn.neighbors import KDTree
import numpy as np
import networkx as nx
from shapely.geometry import Polygon, Point, LineString

from astar import Astar
from sampling import Sampler
from grid import create_grid
import matplotlib.pyplot as plt


data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
sampler = Sampler(data)
polygons = sampler._polygons

# Example: sampling 300 points and removing
# ones conflicting with obstacles.
nodes = sampler.sample(300)
print(len(nodes))


def can_connect(n1, n2):
    l = LineString([n1, n2])
    for p in polygons:
        if p.crosses(l) and p.height >= min(n1[2], n2[2]):
            return False
    return True


def create_graph(nodes, k):
    g = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]

        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue

            if can_connect(n1, n2):
                g.add_edge(n1, n2, weight=1)
    return g

import time
t0 = time.time()
g = create_graph(nodes, 10)
print('graph took {0} seconds to build'.format(time.time()-t0))

grid = create_grid(data, sampler._zmax, 1)
plt.imshow(grid, cmap='Greys', origin='lower')

plt.imshow(grid, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])
emin = np.min(data[:, 1])

# draw edges
for (n1, n2) in g.edges:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black', alpha=0.5)

# draw all nodes
for n1 in nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='blue')

# draw connected nodes
for n1 in g.nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')

plt.xlabel('NORTH')
plt.ylabel('EAST')



start = list(g.nodes)[0]
k = np.random.randint(len(g.nodes))
print(k, len(g.nodes))
goal = list(g.nodes)[k]

print ('start = ', start, ' goal = ', goal)

astar = Astar(g)
found, paths = astar.travel(start, goal)

if found is False: exit("Unable to find path")

path = astar.trace_back(paths)

print(path)

path_pairs = zip(path[:-1], path[1:])
for (n1, n2) in path_pairs:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green')

plt.show()