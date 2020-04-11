import numpy as np
import matplotlib.pyplot as plt
from kalman import KLFilter
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

kl2 = KalmanFilter(dim_x=2, dim_z=2, dim_u=1)

N = 200


def train_noise(var):
    """
    This is the noise that hits the train...
    """
    return np.random.normal(0, scale=np.sqrt(var), size=N)

def get_true_force():
    """
    The true force driving the train:
    """
    return np.cos(np.linspace(0, np.pi*2, N))*F

def train_measurement_noise(var):
    """
    This is the noise due to measurement error
    """
    return np.random.normal(0, scale=np.sqrt(var), size=N)

dt = 1

F = 100
m_train = 10000
var_f = 1
var_x = .01
times = np.arange(N)*dt

input_f = get_true_force()
accel_train = input_f/m_train + train_noise(var_f)/m_train

x_init = 0
v_init = 0
v_train = np.cumsum(accel_train*dt) + v_init
x_train = np.cumsum(v_train)*dt + np.cumsum(accel_train*dt**2) + x_init


train_measured_x = x_train + train_measurement_noise(var_x)
train_measured_v = np.gradient(train_measured_x, times)
train_measured = np.vstack((train_measured_x, train_measured_v)).T

F = np.array([[1, 1], [0, 1]])
P = np.eye(2)*1000
R = np.array([[1, 1/dt], [1/dt, 2/dt**2]])*var_x
B = np.array([[1/2*dt**2], [dt]])
Q = np.array([[1/4*dt**4, 1/2*dt**3], [1/2*dt**3, dt**2]])*var_f/m_train #alpha

init_vec = np.array([0, 1])

kl = KLFilter(init_vec, F=F, P=P, R=R, Q=Q, B=B)
kl2_predict = np.zeros((N, 2))
kl2_estimate = np.zeros((N, 2))
kl_predict = np.zeros((N, 2))
kl_estimate = np.zeros((N, 2))

# Initialize:
kl2_predict[0] = init_vec
kl_predict[0] = init_vec

kl2.x = init_vec
kl2.F = F
kl2.H = np.eye(2)
kl2.P = P
kl2.R = R
kl2.Q = Q
kl2.B = B.ravel()

print(kl2.Q)

for i in range(1, N):
    kl2.predict(input_f[i - 1]/m_train)
    kl2_predict[i] = kl2.x.ravel()
    kl2.update(train_measured[i])
    kl2_estimate[i] = kl2.x.ravel()
    kl_predict[i] = kl.predict(input_f[i-1]/m_train)
    kl_estimate[i] = kl.update(train_measured[i])


_, ax = plt.subplots(figsize=(12, 9))
ax.scatter(times, train_measured_x, marker='+')
ax.plot(times, x_train, lw=1, c='black')
ax.plot(times, kl2_predict[:, 0], label='predict')
ax.plot(times, kl2_estimate[:, 0], label='estim')
ax.plot(times, kl_predict[:, 0], label='predict_T')
ax.plot(times, kl_estimate[:, 0], label='estim_T')
ax.legend()

print(kl2_predict[:5] - kl_predict[:5])
plt.show()
