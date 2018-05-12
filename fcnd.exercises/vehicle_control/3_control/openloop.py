import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from monorotor import Monorotor
import plotting

class OpenLoopController:
    def __init__(self, vehicle_mass, initial_state = np.array([0.0, 0.0])):
        self.vehicle_mass = vehicle_mass
        self.vehicle_state = initial_state
        self.g = 9.81


    def thrust_control(self, target_z, dt):
        """
        Returns a thrust which will be commanded to
        the vehicle. This thrust should cause the vehicle
        to be at target_z in dt seconds.

        The controller's internal model of the vehicle_state
        is also updated in this method.
        """

        current_z, current_z_dot = self.vehicle_state

        delta_z = target_z - current_z
        # velocity needed to reach target postion in dt time
        target_z_dot = delta_z/dt

        delta_z_dot = target_z_dot - current_z_dot
        # acceleration needed to reach target velocity in dt time
        target_z_dot_dot = delta_z_dot/dt

        target_net_force = self.vehicle_mass * target_z_dot_dot
        target_thrust = self.vehicle_mass * self.g - target_net_force

        self.vehicle_state += np.array([delta_z, delta_z_dot])

        return target_thrust


if __name__ == '__main__':
    total_time = 5.0
    t = np.linspace(0.0, total_time, 1000)
    dt = t[1] - t[0]
    z_path = 0.5 * np.cos(2 * t) - 0.5

    MASS_ERROR = 1.01

    drone = Monorotor()
    drone_start_state = drone.X
    drone_mass = drone.m

    # The mass that the controller believes is not necessarily the
    # true mass of the drone. This reflects the real world more accurately.
    perceived_mass = drone_mass * MASS_ERROR
    controller = OpenLoopController(perceived_mass, drone_start_state)

    # 2. Run the simulation
    drone_state_history = []
    for target_z in z_path:
        drone_state_history.append(drone.X)
        thrust = controller.thrust_control(target_z, dt)
        drone.thrust = thrust
        drone.advance_state(dt)

    # 3. Generate plots
    z_actual = [h[0] for h in drone_state_history]
    plotting.compare_planned_to_actual(z_actual, z_path, t)