
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
    subprocess.call(['scp', 'sexybot:bBot/orient.npy', '.'])
    orient_arr = np.load('orient.npy')
elif MODE == 'history':
    files = os.listdir('./datas')
    orient_arr = np.load('datas/' + files[2])

# start of predictions suck...
ndiscard = 10
times = orient_arr[ndiscard:, 0]
phi = orient_arr[ndiscard:, 1]
phidot = orient_arr[ndiscard:, 2]
phidotdot = orient_arr[ndiscard:, 3]

times_pred = orient_arr[ndiscard:, 4]
phi_pred = orient_arr[ndiscard:, 5]
phidot_pred = orient_arr[ndiscard:, 6]
phidotdot_pred = orient_arr[ndiscard:, 7]

vl = orient_arr[ndiscard:, 8]
vr = orient_arr[ndiscard:, 9]
al = orient_arr[ndiscard:, 10]
ar = orient_arr[ndiscard:, 11]

def func(t, a, b, c):
    return a*t**2 + b*t + c

def get_phidot_phidotdot(i, n):

    if PREDICT: n_ = 1
    else: n_ = 0

    if i < n+n_: return 0, 0

    t_ = times[i-n-n_:i-n_]
    phi_ = phi[i-n-n_:i-n_]
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
ax1.plot(times, phi, '-o', markersize = s, label = 'rpi')
ax1.plot(times_pred, phi_pred, '-o', markersize = s, label = 'pred_rpi')
for i in range(len(times)-1):
    ax1.plot([times[i], times_pred[i]], [phi[i], phi_pred[i]], lw = .5, color = 'black')
ax1.legend()
ax1.set_title('phi')
#=======================================================

# phi dot
#=======================================================
ax2.plot(times, phidot, '-o', markersize = s, label = 'rpi')
ax2.plot(times_pred, phidot_pred, '-o', markersize= s, label = 'pred')
for i in range(len(times)-1):
    ax2.plot([times[i], times_pred[i]], [phidot[i], phidot_pred[i]], lw = .5, color = 'black')
ax2.plot(times, np.gradient(phi, times), markersize = s, label = 'grad')
ax2.set_title('phidot')
ax2.legend()
#=======================================================

# Phi dodtot
#=======================================================
ax3.plot(times, phidotdot, '-o', markersize = s, label = 'rpi')
ax3.plot(times_pred, phidotdot_pred, '-o', markersize = s, label = 'pred')
ax3.plot(times, np.gradient(np.gradient(phi, times), times), markersize = s, label = 'grad')
ax3.set_title('phidotdot')
ax3.legend()
#=======================================================

# Vleft and right
#=======================================================
ax4.plot(times, vl, '-o', markersize = s, label = 'vl')
ax4.plot(times, vr, '-o', markersize = s, label = 'vr')

# Vleft and right
#=======================================================
ax5.plot(times, al, '-o', markersize = s, label = 'al')
ax5.plot(times, ar, '-o', markersize = s, label = 'ar')

ax5.set_xlabel('time')
plt.show()

str_time = datetime.datetime.now().strftime("%d-%m_%H:%M")
np.save('datas/run_date-{}.npy'.format(str_time), orient_arr)


plt.plot(times, times_pred, '-o', markersize = s)
plt.plot(times, times, '-', color = 'black')
plt.show()



print(orient_arr[-20:])
