import matplotlib.pyplot as plt
import numpy as np
from modelpatric import get_thetadotdot
from score import score_run
from scipy.optimize import curve_fit

def plot_dynamics(run_data): #, theta_test):

    times = run_data[:, 0]
    thetas = run_data[:, 1]
    thetadots = run_data[:, 2]
    thetadotdots = run_data[:, 3]
    target_thetas = run_data[:, 7]
    target_thetadot = run_data[:, 8]
    target_x = run_data[:, 9]
    target_v = run_data[:, 11]
    target_l = run_data[:, 12]
    run_l = run_data[:, 13]
    xpos = run_data[:, 16]
    vel = run_data[:, 14]
    accel = run_data[:, 15]
    target_a = run_data[:, 19]

    figh = 4
    figw = 10
    fig, axarr = plt.subplots(6, 1, figsize=(figw, figh*3), sharex = True)
    for ax, dat, title in zip(axarr.ravel(),
                              [thetas, thetadots, thetadotdots, run_l, vel, accel],
                              ['theta', 'thetadot', 'thetadotdot', 'run_l', 'vel', 'accel']):
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
    alpha, beta = fit_model(run_data)
    axarr[2].plot(times, get_thetadotdot(thetas, accel, alpha, beta), label='Fitted alpha={:.0f}, beta={:.0f}'.format(alpha, beta))
    # axarr[2].plot(theta_test[:, 0], theta_test[:, 3], label='model_2')
    axarr[2].legend()

    axarr[3].plot(times, target_l, label='target')
    axarr[3].legend()

    axarr[4].plot(times, target_v, label='target')
    axarr[4].legend()

    axarr[5].plot(times, target_a, label='target')
    axarr[5].legend()

    plt.tight_layout()
    fig.suptitle('Score {:.2f}'.format(score_run(run_data)))
    plt.show()


def fit_model(run_data):

    times = run_data[:, 0]
    thetas = run_data[:, 1]
    thetadots = run_data[:, 2]
    thetadotdots = run_data[:, 3]
    accel = run_data[:, 15]

    def get_thetadotdot_wrap(xdat, alpha, beta):
        thetas = xdat[:, 0]
        accel = xdat[:, 1]
        return get_thetadotdot(thetas, accel, alpha, beta)

    xdat = np.vstack((thetas, accel)).T
    alpha, beta = curve_fit(get_thetadotdot_wrap, xdat, thetadotdots, [500, -100])[0]

    return alpha, beta
