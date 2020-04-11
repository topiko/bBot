import numpy as np
import matplotlib.pyplot as plt
from kalman import KLFilter
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

kl2 = KalmanFilter(dim_x=2, dim_z=2)

N = 200

def train_path():
    """
    Define the actual 1D train path.
    """
    #return np.arange(N)*0
    return np.sin(np.linspace(0,np.pi*2, N))

def train_noise(var):
    """
    This is the noise that hits the train...
    """
    return np.random.normal(0, scale=var, size=N)

def train_measurement_noise(var):
    """
    This is the noise due to measurement error
    """
    return np.random.normal(0, scale=var, size=N)

times = np.arange(N)
train_measured_x = train_path() + train_measurement_noise(.01)
train_measured_v = np.append(np.diff(train_measured_x), 0)
train_measured = np.vstack((train_measured_x, train_measured_v)).T

init_x =  np.array([0, 1])
F = np.array([[1, 1], [0, 1]])
P = np.eye(2)*1000
R =  np.array([[.1, 0], [0, .5]])
Q = Q_discrete_white_noise(dim=2, dt=1, var=0.13)*.1

kl = KLFilter(init_x, F=F, P=P, R=R, Q=Q)
kl2_predict = np.zeros((N, 2))
kl2_estimate = np.zeros((N, 2))
kl_predict = np.zeros((N, 2))
kl_estimate = np.zeros((N, 2))
kl_predict[0] = [0, 1]

kl2.x = init_x
kl2.F = F
kl2.H = np.eye(2)
kl2.P = P
kl2.R = R
kl2.Q = Q

print(kl2.Q)

for i in range(1, N):
    kl2.predict()
    kl2_predict[i] = kl2.x.ravel()
    kl2.update(train_measured[i])
    kl2_estimate[i] = kl2.x.ravel()
    kl_predict[i] = kl.predict()
    kl_estimate[i] = kl.update(train_measured[i])


_, ax = plt.subplots(figsize=(12, 9))
ax.scatter(times, train_measured_x, marker='+')
ax.plot(times, train_path(), lw=1, c='black')
ax.plot(times, kl2_predict[:, 0], label='predict')
ax.plot(times, kl2_estimate[:, 0], label='estim')
ax.plot(times, kl_predict[:, 0], label='predict_T')
ax.plot(times, kl_estimate[:, 0], label='estim_T')
ax.legend()

plt.show()
