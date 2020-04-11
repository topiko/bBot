import numpy as np
import matplotlib.pyplot as plt
from kalman import KLFilter
from filterpy.kalman import KalmanFilter


dat = np.load('orient.npy')[3:]
N = len(dat)
dt = np.diff(dat[:, 0]).mean()

theta0 = 15
theta = dat[:, 1] - theta0
thetadot = dat[:, 2]
thetadotdot = dat[:, 3]
a_ground = dat[:, 5]

var_thetadotdot = 25.00
var_theta = .015**2

times = dat[:, 0]

thetadotdot = dat[:, 3]

theta_init = theta[0] #dat[0, 1]
thetadot_init = dat[0, 2]


alpha = 13350
beta = 1026
model_thetadotdot = lambda a, theta: alpha*np.sin(-theta/180*np.pi) + beta*np.cos(-theta/180*np.pi)*a

kalman_input_arr = np.vstack((theta, thetadot)).T

F = np.array([[1, dt],
              [0, 1]])

P = np.eye(2)*1000
R = np.array([[1, 1/dt], [1/dt, 2/dt**2]])*var_theta
B = np.array([[1./2*dt**2],
              [dt]]) #None
Q = np.array([[1/4*dt**4, 1/2*dt**3],
              [1/2*dt**3, dt**2]])*var_thetadotdot

init_vec = np.array([theta_init, thetadot_init])

kl = KLFilter(init_vec, F=F, P=P, R=R, Q=Q, B=B)
kl2_predict = np.zeros((N, 2))
kl2_estimate = np.zeros((N, 2))
kl_predict = np.zeros((N, 2))
kl_estimate = np.zeros((N, 2))

# Initialize:
kl2_predict[0] = init_vec
kl_predict[0] = init_vec

kl2 = KalmanFilter(dim_x=2, dim_z=2, dim_u=1)
kl2.x = init_vec
kl2.F = F
kl2.H = np.eye(2)
kl2.P = P
kl2.R = R
kl2.Q = Q
kl2.B = B.ravel()


for i in range(1, N):
    input_thetadotdot = model_thetadotdot(a_ground[i-1], theta[i-1])
    kl2.predict(input_thetadotdot) #(input_f[i - 1]/m_train)
    kl2_predict[i] = kl2.x.ravel()
    kl2.update(kalman_input_arr[i])
    kl2_estimate[i] = kl2.x.ravel()
    kl_predict[i] = kl.predict(input_thetadotdot) #(input_f[i-1]/m_train)
    kl_estimate[i] = kl.update(kalman_input_arr[i])


_, (ax, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(12, 9))
ax.scatter(times, theta, marker='+')
ax.plot(times, theta, lw=1, c='black')
ax.plot(times, kl2_predict[:, 0], label='predict')
ax.plot(times, kl2_estimate[:, 0], label='estim')
ax.plot(times, kl_predict[:, 0], label='predict_T')
ax.plot(times, kl_estimate[:, 0], label='estim_T')
ax.set_title('theta')
ax.legend()

ax2.plot(times, thetadot, label='measure')
ax2.plot(times, np.gradient(kl_estimate[:, 0], times), label='kl')
ax2.set_ylabel('deg/s')
ax2.set_title('theta.')
ax2.legend()

ax3.plot(times, thetadotdot, label='measure')
ax3.plot(times, np.gradient(kl_estimate[:, 1], times), label='kl')
ax3.plot(times, model_thetadotdot(a_ground, kl_estimate[:, 0]), label='..')
ax3.set_title('theta..')
ax3.set_ylabel('deg/s^2')
ax3.legend()

ax4.plot(times, a_ground, label='a')
ax4.set_title('a ground')
ax4.set_ylabel('m/s^2')
ax4.legend()
plt.show()
