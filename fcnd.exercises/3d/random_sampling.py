import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, Point
from grid import create_grid

def extract_polygons(data):
    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [north - d_north, north + d_north, east - d_east, east + d_east]

        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]
        height = alt + d_alt

        polygon = Polygon(corners)
        polygons.append((polygon, height))

    return polygons

def get_samples(data, sample_count = 100):
    # x axis
    east_min = np.min(data[:, 0] - data[:, 3])
    east_max = np.max(data[:, 0] + data[:, 3])

    # y axis
    north_min = np.min(data[:, 1] - data[:, 4])
    north_max = np.max(data[:, 1] + data[:, 4])

    alt_min = 0
    alt_max = 10

    xvals = np.random.uniform(east_min, east_max, sample_count)
    yvals = np.random.uniform(north_min, north_max, sample_count)
    zvals = np.random.uniform(alt_min, alt_max, sample_count)

    return np.array(list(zip(xvals, yvals, zvals)))

def remove_collides(polygons, samples):
    non_colliding_samples = []
    for point in samples:
        collides = False
        for (polygon, height) in polygons:
            # collides
            if polygon.contains(Point(point)) and point[2] <= height:
                collides = True
                break

        if not collides:
            non_colliding_samples.append(point)

    return non_colliding_samples



data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
polygons = extract_polygons(data)
samples = get_samples(data)

t0 = time.time()
samples = remove_collides(polygons, samples)
#print(samples)
print("Time taken {0} seconds ...".format(time.time() - t0))
print(len(samples))

grid = create_grid(data, 10, 1)
plt.imshow(grid, cmap='Greys', origin='lower')


all_pts = np.array(samples)
north_vals = all_pts[:,0]
east_vals = all_pts[:,1]

north_min = np.min(data[:, 0])
east_min = np.min(data[:, 1])
plt.scatter(east_vals - east_min, north_vals - north_min, c='red')

plt.ylabel('NORTH')
plt.xlabel('EAST')

plt.show()