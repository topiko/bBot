from params import ALPHA, GRAVITY_ACCEL, UPRIGHT_THETA
import numpy as np

def get_thetadotdot(theta, accel, alpha=ALPHA):
    """
    Get the reaction in thetadotdot for given theta and accel
    """
    theta = (theta - UPRIGHT_THETA)/180*np.pi
    #return ALPHA * (GRAVITY_ACCEL * np.sin(theta) - np.cos(theta) * accel) # / np.pi * 180
    return alpha * (GRAVITY_ACCEL * np.sin(theta) - np.cos(theta) * accel) # + beta * accel

