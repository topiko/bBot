"""
This is patrics simulation module.

Intention is so that one can run the balance cyckle as normal
and here we do simulations required.
"""

import numpy as np
from scipy.integrate import solve_ivp
from params import UPRIGHT_THETA, ALPHA, GRAVITY_ACCEL, SIGMA_THETA
import matplotlib.pyplot as plt

class simulate_patric():

    def __init__(self, dt=.016, theta=UPRIGHT_THETA, thetadot=0):
        """
        Init.
        """
        self.dt = dt
        self.theta = theta + .1
        self.thetadot = thetadot
        self.time = 0
        self.alpha = ALPHA #7491 #10

    def run(self, state_dict, cmd_dict):
        """
        Propagate the dynamics by amount dt
        """
        if self.theta != state_dict['theta'][0]:
            raise ValueError('Theta at the simul and the \
                             one in state dict should match. \
                             We have {}i and {}'.format(self.theta, state_dict['theta'][0]))
        accel = cmd_dict['a']
        def theta_derivs(t, theta_vec):
            theta = (theta_vec[0] - UPRIGHT_THETA)/180*np.pi
            thetadot = theta_vec[1]

            thetadotdot = self.alpha * \
                    (GRAVITY_ACCEL * np.sin(theta) - np.cos(theta) * accel)

            return thetadot, thetadotdot

        sol = solve_ivp(theta_derivs,
                        [0, self.dt],
                        np.array([self.theta, self.thetadot]))

        thetas = sol['y'][0]
        thetadots = sol['y'][1]

        # Theta noise:
        theta_noise = np.random.normal(0, scale=SIGMA_THETA)
        self.theta = thetas[-1] + theta_noise
        self.thetadot = thetadots[-1]
        self.time += self.dt

def plot_dynamics(theta_data):

    times = theta_data[:, 0]
    thetas = theta_data[:, 1]
    thetadots = theta_data[:, 2]
    thetadotdots = theta_data[:, 3]
    accel = theta_data[:, 5]

    figh = 4
    figw = 10
    _, axarr = plt.subplots(4, 1, figsize=(figw, figh*3))
    for ax, dat, title in zip(axarr.ravel(),
                              [thetas, thetadots, thetadotdots, accel],
                              ['theta', 'thetadot', 'thetadotdot', 'accel']):
        ax.plot(times, dat, '-+')
        ax.set_title(title)

    plt.show()

def display_simulation(ser, balance_loop):
    """
    This is the main loop plotting the simulation results.
    """
    i = 0
    cmd_dict = None
    state_dict = None
    while True:
        orient_arr, ser, status, cmd_dict, state_dict \
                = balance_loop(ser,
                               run_time_max=3*(i+1),
                               cmd_dict=cmd_dict,
                               state_dict=state_dict)
        print(orient_arr[-1, 1], orient_arr[-1, 2], orient_arr[-1, 0])
        print(ser.theta, ser.thetadot, ser.time)
        plot_dynamics(orient_arr)
        if status == 'fell':
            break
        i += 1


    return
