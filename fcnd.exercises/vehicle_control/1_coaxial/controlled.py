import math
import matplotlib.pyplot as plt
import numpy as np

class CoaxialCopter:
    def __init__(self, k_f = 0.1, k_m = 0.1, m = 0.5, i_z=0.2):
        self.k_f = k_f
        self.k_m = k_m
        self.m = m
        self.i_z = i_z

        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.g = 9.81

        # z, psi, z_dot, psi_dot
        self.X = np.array([0.0, 0.0, 0.0, 0.0])

    @property
    def z_dot_dot(self):
        '''
        Calculates current verticle acceleration
        '''
        Fz1 = - self.k_f * (self.omega_1 ** 2)
        Fz2 = - self.k_f * (self.omega_2 ** 2)
        Fg = self.m * self.g

        return (Fg + Fz1 + Fz2)/self.m

    @property
    def psi_dot_dot(self):
        '''
        Calculates current angular acceleration
        '''

        torque1 = self.k_m * self.omega_1 ** 2
        torque2 = - self.k_m * self.omega_2 ** 2

        return (torque1 + torque2) / self.i_z

    def set_rotors_angular_velocities(self, linear_acc, angular_acc):
        """
        Sets the turn rates for the rotors so that the drone
        achieves the desired linear_acc and angular_acc.
        """

        term_1 = self.m * (-linear_acc + self.g) / (2 * self.k_f)
        term_2 = self.i_z * angular_acc / (2 * self.k_m)

        omega_1 = math.sqrt(term_1 + term_2)
        omega_2 = math.sqrt(term_1 - term_2)

        self.omega_1 = -omega_1
        self.omega_2 = omega_2

        return self.omega_1, self.omega_2

    def advance_state_uncontrollerd(self, dt):
        '''
        Advance state by dt during freefall
        '''
        z_dot_dot = 9.8

        #change in velocity
        delta_z_dot = z_dot_dot * dt
        self.z_dot = self.z_dot + delta_z_dot

        delta_z = self.z_dot * dt
        self.z = self.z + delta_z

        psi_dot_dot = 0.0

        #change in angular velocity
        delta_psi_dot = psi_dot_dot * dt
        self.psi_dot = self.psi_dot + delta_psi_dot

        #change in angular position
        delta_psi = self.psi_dot * dt
        self.psi = self.psi + delta_psi

    def advance_state(self, dt):
        X_dot = np.array([
            self.X[2],          #z_dot
            self.X[3],          #psi_dot
            self.z_dot_dot,
            self.psi_dot_dot
        ])


        delta_X = X_dot * dt
        self.X = self.X + delta_X



t = np.linspace(0.0,15.0,2000)
target_z = np.cos(t) - 1
target_z_dot_dot = -np.cos(t)
dt = t[1] - t[0]

drone = CoaxialCopter()
z_history = []
for z_dot_dot_targ in target_z_dot_dot:
    z_history.append(drone.X[0])
    drone.set_rotors_angular_velocities(z_dot_dot_targ, 0.0)
    drone.advance_state(dt)

plt.plot(t, z_history)
plt.plot(t, target_z, 'r-')
plt.ylabel('Z position meters')
plt.xlabel('Time (s)')
plt.gca().invert_yaxis()
plt.show()