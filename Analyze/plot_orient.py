
import sys
import os
import subprocess
import datetime

import numpy as np
import matplotlib.pyplot as plt
from scipy.misc import derivative
from scipy.optimize import curve_fit
from matplotlib.colors import Normalize

if len(sys.argv) == 2: MODE = sys.argv[1]
else: MODE = 'read_rpi'

PREDICT = True

if MODE == 'read_rpi':
    subprocess.call(['scp', 'sexybot:bBot/Raspberry/orient.npy', '.'])
    orient_arr = np.load('orient.npy')
elif MODE == 'latest':
    orient_arr = np.load('orient.npy')
else:
    files = os.listdir('./datas')
    try:
        orient_arr = np.load('datas/' + MODE)
    except FileNotFoundError:
        print('Invalid fname: {}'.format(MODE))
        orient_arr = np.load('datas/' + files[0])

# start of predictions suck...
ndiscard = 10
nconv = np.ones(5)/5
times = orient_arr[ndiscard:, 0]
times -= times.min()

theta = orient_arr[ndiscard:, 1]
thetadot = orient_arr[ndiscard:, 2]
thetadotdot = orient_arr[ndiscard:, 3]
theta_conv = np.convolve(theta, nconv, mode='same')

theta_measured = orient_arr[ndiscard:, 4]
thetadot_measured = orient_arr[ndiscard:, 5]
thetadotdot_measured = orient_arr[ndiscard:, 6]

times_pred = orient_arr[ndiscard:, 17]
times_pred -= times_pred.min()
theta_pred = orient_arr[ndiscard:, 18]

x = orient_arr[ndiscard:, 14]
y = orient_arr[ndiscard:, 15]

v = orient_arr[ndiscard:, 12]
a = orient_arr[ndiscard:, 13]


def func(t, a, b, c):
    return a*t**2 + b*t + c

def get_phidot_phidotdot(i, n):

    if PREDICT: n_ = 1
    else: n_ = 0

    if i < n+n_: return 0, 0

    t_ = times[i-n-n_:i-n_]
    phi_ = theta[i-n-n_:i-n_]
    (a, b, c), _ = curve_fit(func, t_, phi_, (1, 1, 1))
    phidot = derivative(func, times[i], dx=.05, args=(a, b, c))
    phidotdot = derivative(func, times[i], n=2, dx=.05, args=(a, b, c))
    return phidot, phidotdot

def get_phidot_phidotdot_arr(times, n):

    phidot_arr = np.zeros(len(times))
    phidotdot_arr = np.zeros(len(times))
    for i in range(len(times)):
        phidot, phidotdot = get_phidot_phidotdot(i, n)
        phidot_arr[i] = phidot
        phidotdot_arr[i] = phidotdot

    return phidot_arr, phidotdot_arr

def get_thetadotdot_model(x, alpha):
    a = x[:, 0]
    theta = x[:, 1]/180*np.pi
    g = 9.81
    theta0 = 15
    theta0 = theta0/180*np.pi
    return alpha*(g*np.sin(theta - theta0) - np.cos(theta-theta0)*a)
    #return beta*np.cos(theta -theta0)*a + alpha*np.sin(theta - theta0)
    #return alpha*np.sin(theta-theta0) + beta*np.cos(theta-theta0)*a


# phi, phidotdot, a
def get_thetadotdot_model_old(x, R, theta0):
    a = x[:, 0]
    theta = x[:, 1]/180*np.pi
    g = 9.81
    theta0 = theta0/180*np.pi
    return -1/R*(g*np.sin(theta0-theta) + np.cos(theta0-theta)*a)/np.pi*180

fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, figsize=(12,8), sharex=True)
fig2, ax_loc = plt.subplots(figsize=(12,12))

s = 5
# Phi:
#=======================================================
ax1.plot(times, theta, '-o', markersize=s, label='rpi')
ax1.plot(times, theta_measured, '-*', markersize=s, label='measured')
ax1.plot(times_pred, theta_pred, '-o', markersize=s, label='pred_rpi')
#ax1.plot(times, theta_conv, '-', markersize=s, label='conv')
#for i in range(len(times)-1):
#    ax1.plot([times[i], times_pred[i]], [theta[i], theta_pred[i]], lw = .5, color = 'black')
ax1.legend()
ax1.set_title('theta')
#=======================================================

# phi dot
#=======================================================
ax2.plot(times, thetadot, '-o', markersize=s, label='rpi')
#for i in range(len(times)-1):
#    ax2.plot([times[i], times_pred[i]], [thetadot[i], thetadot_pred[i]], lw = .5, color = 'black')
ax2.plot(times, thetadot_measured, '-*', markersize=s, label='measured')
ax2.plot(times, np.gradient(theta, times), markersize=s, label='grad theta')
ax2.set_title('theta.')
ax2.legend()
#=======================================================

# Phi dodtot
#=======================================================
gradv = np.gradient(v, times)
fit_dat = np.hstack((gradv.reshape(-1, 1), theta.reshape(-1, 1)))
gradgradtheta = np.gradient(np.gradient(theta, times), times)
alpha = curve_fit(get_thetadotdot_model, fit_dat, gradgradtheta, [.1],
                        bounds=(0, np.inf))[0][0]
print(alpha)
thetadotdot_model = get_thetadotdot_model(fit_dat, alpha)

ax3.plot(times, thetadotdot, '-o', markersize=s, label='rpi')
ax3.plot(times, thetadotdot_measured, '-*', markersize=s, label='measured')
ax3.plot(times, thetadotdot_model, '-o', markersize=s,
         label=r'model, alpha={:.02f}'.format(alpha))
#ax3.plot(times, gradgradtheta, label='grad grad theta')
ax3.set_title('theta..')
ax3.legend()
#=======================================================

# Vleft and right
#=======================================================
ax4.plot(times, v, '-o', markersize=s, label='v')
ax4.set_ylabel('v [m/s]')

# Vleft and right
#=======================================================
ax5.plot(times, a, '-o', markersize=s, label='a')
ax5.plot(times, gradv, '-o', markersize=s, label='grad v')
ax5.legend()
ax5.set_ylabel('a [m/s^2]')
ax5.set_xlabel('time')


# Location
ax_loc.plot(x*1000, y*1000, '-+', c='black', lw=1)
ax_loc.axis('equal')
ax_loc.set_xlabel('x [mm]')
ax_loc.set_ylabel('y [mm]')
plt.show()


# Test model between phi.., phi and a
x1 = gradv
x2 = theta
Y = thetadotdot
_, ax_2d = plt.subplots(figsize=(10, 6))
ax_2d.tricontour(x1, x2, Y, 5, linewidths=0.2)
norm = Normalize(vmin=-3000, vmax=3000, clip=False)
cntr1 = ax_2d.tricontourf(x1, x2, Y, 100, cmap="RdBu_r", norm = norm)
ax_2d.scatter(x1, x2, s=5, alpha=.3)

ax_2d.set_xlabel('a [m/s^2]')
ax_2d.set_ylabel('theta [deg]')
fig.colorbar(cntr1, ax=ax_2d, label='theta..')
plt.show()

if MODE == 'read_rpi':
    str_time = datetime.datetime.now().strftime("%d-%m_%H:%M")
    np.save('datas/run_date-{}.npy'.format(str_time), orient_arr)




