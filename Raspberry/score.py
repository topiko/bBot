"""
MOdule to handle the performance scoring of Patric.
"""

#from scipy.integrate import trapz
import numpy as np

def score_run(run_array):
    """
    Obtain a performance score given from run array.
    """
    times = run_array[:, 0]
    thetas = run_array[:, 1]
    target_thetas = run_array[:, 7]

    squared_diff = (thetas - target_thetas)**2
    return (squared_diff[1:]*np.diff(times)).sum()/(times[-1] - times[0])
    #return trapz(squared_diff, times)/(times[-1] - times[0])
