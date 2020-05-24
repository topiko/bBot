"""
Implement simple Kalman filter.
"""
import numpy as np
from params import SIGMA_THETA, SIGMA_THETADOTDOT

def get_patric_kalman(init_state, dt):
    """
    Get kalman filter object that is tuned for patric
    """
    F = np.array([[1, dt],
                  [0, 1]])

    P = np.eye(2)*1000
    R = np.array([[1, 1/dt],
                  [1/dt, 2/dt**2]])*SIGMA_THETA**2
    B = np.array([[1./2*dt**2],
                  [dt]]) #None
    Q = np.array([[1/4*dt**4, 1/2*dt**3],
                  [1/2*dt**3, dt**2]])*SIGMA_THETADOTDOT**2


    return  KLFilter(init_state, F=F, P=P, R=R, Q=Q, B=B)


class KLFilter():
    """
    Very simple Kalman filter.
    """
    def __init__(self, x, F=None, P=None, R=None, Q=None, B=None):

        n = len(x)
        self.n = n
        self.x = x.reshape(-1, 1)
        try:
            kl_matrx = np.load('kl_mats.npy', allow_pickle=True).item()
            P = kl_matrx['P']
            F = kl_matrx['F']
        except FileNotFoundError:
            pass


        self.P = P #if P is not None else np.eye(n)*1000
        self.Q = Q #np.array([[.25, .5], [.5, .1]]) if Q is None else Q
        self.F = F #if F is not None else np.eye(n)
        self.K = np.eye(n)

        self.R = R #np.diag([.1, .1*np.sqrt(2)]) if R is None else R
        self.H = np.eye(2) #np.array([[1, 0], [0, 1]])
        self.B = B #np.zeros(self.x.shape) if B is None else B

    def store(self):
        kl_matrx = {'P':self.P, 'F':self.F}
        np.save('kl_mats.npy', kl_matrx)

    def predict(self, control_input=None):
        """
        Predict the state.
        """

        if control_input is not None:
            control_effect = self.B*control_input
        else:
            control_effect = np.zeros(self.x.shape)

        self.x = np.matmul(self.F, self.x) + control_effect  #self.F.dot(self.x)
        self.P = np.matmul(self.F, np.matmul(self.P, self.F.T)) + self.Q

        return self.x.ravel()

    def update(self, z):
        """
        Update the state after a measurement (z) is received.
        """
        if z.shape[0] == self.n:
            z = z.reshape(-1, 1)

        mat1 = np.matmul(self.H, np.matmul(self.P, self.H.T))
        mat2 = np.linalg.inv(mat1 + self.R)
        self.K = np.matmul(self.P, np.matmul(self.H.T, mat2))

        self.P = self.P - np.matmul(self.K, np.matmul(self.H, self.P))
        self.x = self.x + np.matmul(self.K, z - np.matmul(self.H, self.x))

        return self.x.ravel()
