"""
This is patrics simulation module.

Intention is so that one can run the balance cyckle as normal
and here we do simulations required.
"""

import numpy as np
from scipy.integrate import solve_ivp

class simulate_patric():

    def __init__(self, dt = .016, theta=0, thetadot=0):
        """
        Init.
        """
        self.dt = dt
        self.theta = theta
        self.thetadot = thetadot
        self.time = 0
        # TODO: fix these:
        self.alpha = 10
        self.beta = 20

    def run(self, state_dict, cmd_dict):
        """
        Propagate the dynamics by amount dt
        """
        if self.theta != state_dict['theta']:
            raise ValueError('Theta at the simul and the \
                             one in state dict should match.')

        accel = cmd_dict['a']
        def theta_derivs(t, theta_vec):
            theta = theta_vec[0]
            thetadot = theta_vec[1]

            thetadotdot = self.alpha*np.sin(theta) \
                    + self.beta*np.cos(theta)

            return thetadot, thetadotdot

        sol = solve_ivp(theta_derivs,
                        [0, self.dt],
                        np.array([self.theta, self.thetadot]))

        thetas = sol['y'][0]
        thetadots = sol['y'][1]

        self.theta = thetas[-1]
        self.thetadot = thetadots[-1]
        self.time += self.dt

def display_simulation():
    """
    This is the main loop plotting the simulation results.
    """

    return

