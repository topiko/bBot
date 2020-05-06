"""
MOdule to handle the performance scoring of Patric.
"""

from scipy.integrate import trapz
import numpy as np

def score_run(run_array):
    """
    Obtain a performance score given from run array.
    """
    if run_array is None:
        return np.inf

    times = run_array[:, 0]
    thetas = run_array[:, 1]
    target_thetas = run_array[:, 7]
    run_l = run_array[:, 13]
    target_l = run_array[:, 12]

    tot_time = times[-1] - times[0]
    score = 0
    for vals, target_vals in [#(thetas, target_thetas),
                              (run_l, target_l)]:
        squared_diff = (vals - target_vals)**2
        score += trapz(squared_diff, times)/tot_time

    return score
    #(squared_diff[1:]*np.diff(times)).sum()/(times[-1] - times[0])
    #return trapz(squared_diff, times)/(times[-1] - times[0])
