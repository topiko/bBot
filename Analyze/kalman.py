"""
Implement simple Kalman filter.
"""
import numpy as np

class KLFilter():
    """
    Very simple Kalman filter.
    """
    def __init__(self, x, F=None, P=None, R=None, Q=None, B=None):

        dt = 1
        n = len(x)
        self.n = n
        self.x = x.reshape(-1, 1)
        self.P = P if P is not None else np.eye(n)*1000
        self.Q = np.array([[.25, .5], [.5, .1]]) if Q is None else Q
        self.F = F if F is not None else np.eye(n)
        self.K = np.eye(n)
        self.R = np.diag([.1, .1*np.sqrt(2)]) if R is None else R
        self.H = np.array([[1, 0], [0, 1]])
        self.B = np.zeros(self.x.shape) if B is None else B

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
        #print('P = ', self.P)

        mat1 = np.matmul(self.H, np.matmul(self.P, self.H.T))
        mat2 = np.linalg.inv(mat1 + self.R)
        self.K  = np.matmul(self.P, np.matmul(self.H.T, mat2))

        self.P = self.P - np.matmul(self.K, np.matmul(self.H, self.P))
        #print('z = ', z, 'estim x = ', self.x)
        self.x = self.x + np.matmul(self.K, z - np.matmul(self.H, self.x))
        #print('K:', self.K)
        #print('P:', self.P)
        #print()
        return self.x.ravel()
