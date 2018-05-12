import numpy as np 
import time

def to3d(point):
    return np.array([point[0], point[1], 1.0])

def determinent(p1, p2, p3):
    mat = np.vstack((to3d(p1), to3d(p2), to3d(p3)))
    print(mat)
    return np.linalg.det(mat)

def simple_determinent(p1, p2, p3):
    return p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])

p1 = (1, 2)
p2 = (2, 3)
p3 = (3, 4)

t1 = time.time()
det = determinent(p1, p2, p3)
print('time taken = ', time.time() - t1)
epsilon = 1e-2

det = 0 if det < epsilon else det
print(det)

t1 = time.time()
det = simple_determinent(p1, p2, p3)
print('time taken = ', time.time() - t1)
epsilon = 1e-2

det = 0 if det < epsilon else det
print(det)

