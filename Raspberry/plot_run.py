import matplotlib.pyplot as plt
import numpy as np
from modelpatric import get_thetadotdot
from score import score_run
from scipy.optimize import curve_fit
from params import ALPHA
import os

def plot_dynamics(run_data): #, theta_test):

    times = run_data['times']
    thetas = run_data['theta']
    thetadots = run_data['thetadot']
    thetadotdots = run_data['thetadotdot']
    target_thetas = run_data['target_theta']
    target_thetadot = run_data['target_thetadot']
    target_thetadotdot = run_data['target_thetadotdot']
    target_x = run_data['target_x']
    target_v = run_data['target_v']
    target_l = run_data['target_l']
    run_l = run_data['run_l']
    xpos = run_data['x']
    vel = run_data['v']
    accel = run_data['a']
    a1 = run_data['a1']
    a2 = run_data['a2']
    target_a = run_data['target_a']

    figh = 4
    figw = 10
    fig, axarr = plt.subplots(6, 1, figsize=(figw, figh*3), sharex = True)
    for ax, dat, title in zip(axarr.ravel(),
                              [thetas, thetadots, thetadotdots, run_l, vel, accel],
                              ['theta', 'thetadot', 'thetadotdot', 'run_l', 'vel', 'accel']):
        ax.plot(times, dat, '-+')
        ax.set_title(title)
        ax.grid(True)

    axarr[0].plot(times, run_data['theta_measured'], label='measured', lw=.5, alpha=.5)
    axarr[0].plot(times, target_thetas, label='targeted')
    axarr[0].legend()

    axarr[1].plot(times, run_data['thetadot_measured'], label='measured', lw=.5, alpha=.5)
    axarr[1].legend()

    axarr[2].plot(times, run_data['thetadotdot_measured'], label='measured', lw=.5, alpha=.5)
    #axarr[2].plot(times, get_thetadotdot(thetas, accel), label='model')
    axarr[2].plot(times, target_thetadotdot, label='targeted')

    alpha, beta = fit_model(run_data)
    axarr[2].plot(times,
                  get_thetadotdot(thetas, accel, alpha),
                  label='Fitted alpha={:.1f}'.format(alpha))
    # axarr[2].plot(theta_test[:, 0], theta_test[:, 3], label='model_2')
    axarr[2].plot(times,
                  get_thetadotdot(thetas, accel, ALPHA),
                  label='CUR alpha={:.1f}'.format(ALPHA))

    axarr[2].legend()

    axarr[3].plot(times, target_l, label='target')
    axarr[3].legend()

    axarr[4].plot(times, target_v, label='target')
    axarr[4].legend()

    axarr[5].plot(times, target_a, label='target')
    #axarr[5].plot(times, np.gradient(vel, times), label='grad_v')
    axarr[5].plot(times, a1, label='a1')
    axarr[5].plot(times, a2, label='a2')
    axarr[5].plot(times, a1+a2, label='a1+a2')

    axarr[5].legend()

    plt.tight_layout()
    fig.suptitle('Score {:.2f}'.format(score_run(run_data)))
    plt.show()

    print('Time')
    print(np.diff(times).mean(), np.diff(times).std())

def fit_model(run_data):

    times = run_data['times'] #[:, times]
    thetas = run_data['theta'] #[:, 1]
    thetadots = run_data['thetadot'] #[:, 2]
    thetadotdots = run_data['thetadotdot'] #[:, 3]
    accel = run_data['a'] #[:, 16]

    def get_thetadotdot_wrap(xdat, alpha, beta):
        thetas = xdat[:, 0]
        accel = xdat[:, 1]
        return get_thetadotdot(thetas, accel, alpha) #, beta)

    xdat = np.vstack((thetas, accel)).T
    alpha, beta = curve_fit(get_thetadotdot_wrap, xdat, thetadotdots, [500, -100])[0]

    return alpha, beta

def fit_model_all_data():

    run_data = None
    path = os.getcwd() + '/datas/'
    for f in os.listdir(path):
        if f.startswith('run_date'):
            data = np.load(path + f)
            if run_data is None:
                run_data = data
            else:
                run_data = np.concatenate((run_data, data))
            plot_dynamics(data)

    alpha, beta = fit_model(run_data)

    print('ALPHA = {:.2f}'.format(alpha))

if __name__ == '__main__':
    print('Run as main.')
    fit_model_all_data()
