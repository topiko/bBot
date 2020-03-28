
import sys
import os
import subprocess
import datetime

import numpy as np
import matplotlib.pyplot as plt
from scipy.misc import derivative
from scipy.optimize import curve_fit

if len(sys.argv) == 2: MODE = sys.argv[1]
else: MODE = 'read_rpi'

PREDICT = True

if MODE == 'read_rpi':
    subprocess.call(['scp', 'sexybot:bBot/Raspberry/orient.npy', '.'])
    orient_arr = np.load('orient.npy')
elif MODE == 'history':
    files = os.listdir('./datas')
    orient_arr = np.load('datas/' + files[0])

# start of predictions suck...
ndiscard = 10
times = orient_arr[ndiscard:, 0]
theta = orient_arr[ndiscard:, 1]
thetadot = orient_arr[ndiscard:, 2]
thetadotdot = orient_arr[ndiscard:, 3]

times_pred = orient_arr[ndiscard:, 9]
theta_pred = orient_arr[ndiscard:, 10]

print(times[-10:])
print(times_pred[-10:])
#thetadot_pred = orient_arr[ndiscard:, 6]
#thetadotdot_pred = orient_arr[ndiscard:, 7]

v = orient_arr[ndiscard:, 4]
#vr = orient_arr[ndiscard:, 9]
a = orient_arr[ndiscard:, 5]
#ar = orient_arr[ndiscard:, 11]

def func(t, a, b, c):
    return a*t**2 + b*t + c

def get_phidot_phidotdot(i, n):

    if PREDICT: n_ = 1
    else: n_ = 0

    if i < n+n_: return 0, 0

    t_ = times[i-n-n_:i-n_]
    phi_ = theta[i-n-n_:i-n_]
    (a,b,c), _ = curve_fit(func, t_, phi_, (1,1,1))
    phidot = derivative(func, times[i], dx = .05, args = (a,b,c))
    phidotdot = derivative(func, times[i], n = 2, dx = .05, args =(a,b,c))
    return phidot, phidotdot

def get_phidot_phidotdot_arr(times, n):

    phidot_arr = np.zeros(len(times))
    phidotdot_arr = np.zeros(len(times))
    for i in range(len(times)):
        phidot, phidotdot = get_phidot_phidotdot(i, n)
        phidot_arr[i] = phidot
        phidotdot_arr[i] = phidotdot

    return phidot_arr, phidotdot_arr

fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, figsize = (12,8), sharex = True)

s = 5
# Phi:
#=======================================================
ax1.plot(times, theta, '-o', markersize = s, label = 'rpi')
ax1.plot(times_pred, theta_pred, '-o', markersize = s, label = 'pred_rpi')
#for i in range(len(times)-1):
#    ax1.plot([times[i], times_pred[i]], [theta[i], theta_pred[i]], lw = .5, color = 'black')
ax1.legend()
ax1.set_title('phi')
#=======================================================

# phi dot
#=======================================================
ax2.plot(times, thetadot, '-o', markersize = s, label = 'rpi')
#for i in range(len(times)-1):
#    ax2.plot([times[i], times_pred[i]], [thetadot[i], thetadot_pred[i]], lw = .5, color = 'black')
ax2.plot(times, np.gradient(theta, times), markersize = s, label = 'grad')
ax2.set_title('phidot')
ax2.legend()
#=======================================================

# Phi dodtot
#=======================================================
ax3.plot(times, thetadotdot, '-o', markersize = s, label = 'rpi')
ax3.plot(times, np.gradient(np.gradient(theta, times), times), markersize = s, label = 'grad')
ax3.set_title('phidotdot')
ax3.legend()
#=======================================================

# Vleft and right
#=======================================================
ax4.plot(times, v, '-o', markersize=s, label='v')

# Vleft and right
#=======================================================
ax5.plot(times, a, '-o', markersize=s, label='a')
ax5.set_xlabel('time')

plt.show()

str_time = datetime.datetime.now().strftime("%d-%m_%H:%M")
np.save('datas/run_date-{}.npy'.format(str_time), orient_arr)

plt.plot(times, times_pred, '-o', markersize = s)
plt.plot(times, times, '-', color = 'black')
plt.show()



print(orient_arr[-20:])
