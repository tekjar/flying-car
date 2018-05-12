import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from monorotor import Monorotor
import plotting
import testing
import trajectories

class PController:
    def __init__(self, k_p, m):
        self.k_p = k_p
        self.vehicle_mass = m
        self.g = 9.81

    def thrust_control(self, z_target, z_actual):
        error = z_target - z_actual
        # u_bar is what we want vertical acceleration to be
        u_bar = self.k_p * error

        # u is the thrust command which will cause u_bar
        u = self.vehicle_mass * (self.g - u_bar)

        return u

#testing.p_controller_test(PController)
np.set_printoptions(threshold=np.inf)
np.set_printoptions(suppress=True)

MASS_ERROR = 1.01
K_P = 0.1

# preparation
drone = Monorotor()
perceived_mass = drone.m * MASS_ERROR
controller = PController(K_P, perceived_mass)

# generate trajectory
total_time = 30.0
dt = 0.001
t = np.linspace(0.0, total_time, int(total_time / dt))
z_path = -np.ones(t.shape[0])

# run simulation
history = []
table = []
headers = ["time", "z", "error", "thrust"]
for count, z_target in enumerate(z_path):
    time = count * dt
    z_actual = drone.z
    u = controller.thrust_control(z_target, z_actual)
    drone.thrust = u
    drone.advance_state(dt)
    table.append([time, z_actual, z_target - z_actual, u])
    history.append(drone.X)

print(np.matrix(table))

# generate plots
z_actual = [h[0] for h in history]
plotting.compare_planned_to_actual(z_actual, z_path, t)