import numpy as np 
import matplotlib.pyplot as plot

plot.rcParams['figure.figsize'] = 12, 12

def bresenham(p1, p2): 
    """
    Note this solution requires `x1` < `x2` and `y1` < `y2`.
    """
    x1, y1 = p1
    x2, y2 = p2
    cells = []

    slope = (y2 - y1) * 1.0/(x2 - x1)

    grid_current_y = y1
    grid_current_x = x1

    while (grid_current_x, grid_current_y) != (x2, y2):
        cells.append((grid_current_x, grid_current_y))

        grid_next_y = grid_current_y + 1
        line_next_y = grid_current_x + slope

        #if actual line is greater than grid boundary, else increment x
        if line_next_y >= grid_next_y:
            grid_current_y = grid_current_y + 1
        else:
            grid_current_x = grid_current_x + 1

        print(grid_current_x, grid_current_y)
        
    return np.array(cells)



p1 = (0, 0)
p2 = (5, 4)

cells = bresenham(p1, p2)
print(cells)

plot.plot([p1[0], p2[0]], [p1[1], p2[1]])


for q in cells:
    plot.plot([q[0], q[0]+1], [q[1], q[1]], 'k')
    plot.plot([q[0], q[0]+1], [q[1]+1, q[1]+1], 'k')
    plot.plot([q[0], q[0]], [q[1],q[1]+1], 'k')
    plot.plot([q[0]+1, q[0]+1], [q[1], q[1]+1], 'k')

plot.grid()
plot.axis('equal')
plot.xlabel("X")
plot.ylabel("Y")
plot.title("Integer based Bresenham algorithm")
plot.show()