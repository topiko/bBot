"""
This is patrics simulation module.

Intention is so that one can run the balance cyckle as normal
and here we do simulations required.
"""

import numpy as np
from scipy.integrate import solve_ivp
from params import UPRIGHT_THETA, ALPHA, \
        GRAVITY_ACCEL, SIGMA_THETA, SIMUL_LOOP_TIME
import matplotlib.pyplot as plt
from modelpatric import get_thetadotdot
from score import score_run

class simulate_patric():

    def __init__(self, theta=UPRIGHT_THETA, thetadot=0, dt=.01):
        """
        Init.
        """
        self.theta = theta + 15
        self.thetadot = thetadot
        self.time = 0
        self.dt = dt
        self.noise = True #False #True #False
        self.theta_noisy = self.theta
        self.record_theta = np.zeros((int(SIMUL_LOOP_TIME//self.dt)+3, 4))
        self.idx = 0

    def reset_idx(self):
        self.idx = 0

    def run(self, state_dict, cmd_dict):
        """
        Propagate the dynamics by amount dt
        """
        accel = cmd_dict['a']
        self.dt = state_dict['dt']
        def theta_derivs(t, theta_vec):
            theta = theta_vec[0]
            thetadot = theta_vec[1]

            thetadotdot = get_thetadotdot(theta, accel)
            return thetadot, thetadotdot

        sol = solve_ivp(theta_derivs,
                        [0, self.dt],
                        np.array([self.theta, self.thetadot]))

        thetas = sol['y'][0]
        thetadots = sol['y'][1]

        # Theta noise:
        theta_noise = np.random.normal(0, scale=SIGMA_THETA) if self.noise else 0
        self.theta = thetas[-1]
        self.theta_noisy = self.theta + theta_noise
        self.thetadot = thetadots[-1]
        self.time += self.dt

        #self.record_theta[self.idx] = [self.time, self.theta, self.thetadot, get_thetadotdot(self.theta, accel)]
        #self.idx += 1

def plot_dynamics(run_data): #, theta_test):

    times = run_data[:, 0]
    thetas = run_data[:, 1]
    thetadots = run_data[:, 2]
    thetadotdots = run_data[:, 3]
    target_thetas = run_data[:, 7]
    target_thetadot = run_data[:, 8]
    xpos = run_data[:, 11]
    vel = run_data[:, 9]
    accel = run_data[:, 10]

    figh = 4
    figw = 10
    _, axarr = plt.subplots(6, 1, figsize=(figw, figh*3), sharex = True)
    for ax, dat, title in zip(axarr.ravel(),
                              [thetas, thetadots, thetadotdots, xpos, vel, accel],
                              ['theta', 'thetadot', 'thetadotdot', 'xpos', 'vel', 'accel']):
        ax.plot(times, dat, '-+')
        ax.set_title(title)


    axarr[0].plot(times, run_data[:, 4], label='measured')
    axarr[0].plot(times, target_thetas, label='targeted')
    axarr[0].legend()

    axarr[1].plot(times, run_data[:, 5], label='measured')
    axarr[1].plot(times, target_thetadot, label='targeted')
    axarr[1].legend()

    axarr[2].plot(times, run_data[:, 6], label='measured')
    axarr[2].plot(times, get_thetadotdot(thetas, accel), label='model')
    # axarr[2].plot(theta_test[:, 0], theta_test[:, 3], label='model_2')
    axarr[2].legend()

    plt.tight_layout()
    plt.show()

def display_simulation(ser, balance_loop):
    """
    This is the main loop plotting the simulation results.
    """
    i = 0
    cmd_dict = None
    state_dict = None
    kl = None
    dt = .01
    while True:
        run_array, ser, status, cmd_dict, state_dict, kl \
                = balance_loop(ser,
                               run_time_max=SIMUL_LOOP_TIME*(i+1),
                               cmd_dict=cmd_dict,
                               state_dict=state_dict,
                               kl=kl)
        ser.reset_idx()

        print('SCORE:')
        print(score_run(run_array))

        plot_dynamics(run_array) #, ser.record_theta[:ser.idx])

        if status == 'fell':
            break
        i += 1


    return
