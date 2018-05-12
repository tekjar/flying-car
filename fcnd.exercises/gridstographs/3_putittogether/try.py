from enum import Enum
import matplotlib.pyplot as plt
import numpy as np

class Action(Enum):
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)

    def __str__(self):
        '''
            returns string representation of this action
        '''
        if self == self.LEFT:
            return '◀'
        elif self == self.RIGHT:
            return '▶'
        elif self == self.UP:
            return '▲'
        elif self == self.DOWN:
            return '▼'

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

plt.rcParams['figure.figsize'] = 12, 12

path = []
path.append(Action.DOWN)
path.append(Action.DOWN)
path.append(Action.RIGHT)

x, y = [], []

for action in path:


pp = np.array(path)
x = [x.value[0] for x in pp]
print(x)

# plt.plot(pp[:, 1], pp[:, 0], 'g')

# plt.xlabel('EAST')
# plt.ylabel('NORTH')
# plt.show()

