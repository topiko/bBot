import numpy as np
from kalman import KLFilter
from filterpy.kalman import KalmanFilter

kl2 = KalmanFilter(dim_x=2, dim_z=2)

data = np.load('orient.npy')[:500]

dt = .016
get_F = lambda dt:  np.array([[1, dt], [0, 1]])

times = data[:, 0]
dts = np.diff(times)
theta_arr = data[:, 1:3]

theta_init = theta_arr[0, :]
kl = KLFilter(theta_init, F=get_F(dts[0]))
kl2.x = theta_init
kl2.F = get_F(1)


kl_predict = np.zeros(theta_arr.shape)
kl_measure = np.zeros(theta_arr.shape)

for i, theta in enumerate(theta_arr):
    if i == 0: continue
    kl.F = get_F(dts[i-1])
    kl_predict[i] = kl.predict()
    kl_measure[i] = kl.update(theta_arr[i, :])

#for i in range(len(theta_arr)-1):
#    kl.F = get_F(dts[i])
#    kl_predict[i] = kl.predict()
#    kl_measure[i] = kl.update(theta_arr[i, :])

import matplotlib.pyplot as plt

_, axarr = plt.subplots(3, 1)
for ax, dat, dat_pred, dat_measure in zip(axarr,
                                          theta_arr.T,
                                          kl_predict.T,
                                          kl_measure.T,
                                          ):
    ax.plot(np.arange(len(dat)), dat, '-o', label='measured')
    ax.plot(np.arange(len(dat)), dat_pred, '-*', label='predicted')
    ax.plot(np.arange(len(dat)), dat_measure, '-+', label='estimate')
    ax.legend()

plt.show()
