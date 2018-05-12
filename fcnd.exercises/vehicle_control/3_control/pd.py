import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from monorotor import Monorotor
import plotting
import testing
import trajectories


class PDController:
    def __init__(self, k_p, k_d, m):
        self.k_p = k_p
        self.k_d = k_d
        self.vehicle_mass = m
        self.g = 9.81
        self.last_error = 0

    def thrust_control(self,
                       z_target,
                       z_actual,
                       z_dot_target,
                       z_dot_actual,
                       z_dot_dot_ff=0.0):

        error = z_target - z_actual
        error_dot = z_dot_target - z_dot_actual

        self.last_error = error

        # u_bar is what we want vertical acceleration to be
        u_bar = self.k_p * error + self.k_d * error_dot + z_dot_dot_ff

        #u is the thrust command which will cause u_bar
        u = self.vehicle_mass * (self.g - u_bar)

        return u

testing.pd_controller_test(PDController)


def oscillating_target():
    '''
    returns sampling time, target path, target path velocity, target path acceleration
    '''

    AMPLITUDE = 0.5
    OSCILLATION_FREQUENCY = 5

    PERIOD = 2 * np.pi / OSCILLATION_FREQUENCY

    t, z_path, z_dot_path, z_dot_dot_path = trajectories.cosine(AMPLITUDE,
                                                                PERIOD,
                                                                duration=6.0)

    return t, z_path, z_dot_path, z_dot_dot_path


def constant_target():
    '''
    returns sampling time, target path, target path velocity
    '''
    t, z_path, z_dot_path, z_dot_dot_path = trajectories.step()

    return t, z_path, z_dot_path, z_dot_dot_path

def proceed(monorotor, controller, target_func, feed_forward = False):
    '''
    :param monorotor, controller
    :param target_func function which generates target path and its related
    :returns traversed path by monorotor
    '''
    t, z_path, z_dot_path, z_dot_dot_path = target_func()
    dt = t[1] - t[0]

    history = []
    for z_target, z_dot_target, z_dot_dot_ff in zip(z_path, z_dot_path, z_dot_dot_path):
        z_actual = monorotor.z
        z_dot_actual = monorotor.z_dot

        u = controller.thrust_control(z_target, z_actual, z_dot_target, z_dot_actual, z_dot_dot_ff) if feed_forward else controller.thrust_control(z_target, z_actual, z_dot_target, z_dot_actual)
        monorotor.thrust = u

        monorotor.advance_state(dt)
        history.append(monorotor.X)

    actual_path = [h[0] for h in history]

    return t, z_path, actual_path

if __name__ == '__main__':
    MASS_ERROR = 1.0
    K_P = 20.0
    K_D = 2.0

    drone = Monorotor()
    controller = PDController(K_P, K_D, drone.m * MASS_ERROR)

    t, target_path, actual_path = proceed(drone, controller, oscillating_target)

    drone = Monorotor()
    controller = PDController(K_P, K_D, drone.m * MASS_ERROR)

    t, target_path, actual_path_ff = proceed(drone, controller, oscillating_target, True)
    plotting.compare_planned_to_actual(actual_path, target_path, t, actual_path_ff)


# This code simulates TWO drones. One uses the feed forward
# acceleration and the other doesn't. Note the difference in
# trajectories.