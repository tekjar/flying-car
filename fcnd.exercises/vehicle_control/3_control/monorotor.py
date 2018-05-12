import numpy as np

class Monorotor:
    def __init__(self, m = 1.0):
        self.m = m
        self.g = 9.81
        self.thrust = 0.0

        # z, z_dot
        self.X = np.array([0.0, 0.0])

    @property
    def z(self):
        return self.X[0]

    @property
    def z_dot(self):
        return self.X[1]

    @property
    def z_dot_dot(self):
        return (self.m * self.g - self.thrust) / self.m

    def advance_state(self, dt):
        x_dot = np.array([self.z_dot, self.z_dot_dot])
        self.X = self.X + x_dot * dt
        return self.X
