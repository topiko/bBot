"""
MOdule to handle the performance scoring of Patric.
"""

from scipy.integrate import trapz
import numpy as np

def score_run(run_array):
    """
    Obtain a performance score given from run array.
    """
    times = run_array[:, 0]
    thetas = run_array[:, 1]
    target_thetas = run_array[:, 7]
    x = run_array[:, 14]
    target_x = run_array[:, 9]

    tot_time = times[-1] - times[0]
    score = 0
    for vals, target_vals in [(thetas, target_thetas)]: #,
#                              (x, target_x)]:
        squared_diff = (vals - target_vals)**2
        score += trapz(squared_diff, times)/tot_time

    return score
    #(squared_diff[1:]*np.diff(times)).sum()/(times[-1] - times[0])
    #return trapz(squared_diff, times)/(times[-1] - times[0])
