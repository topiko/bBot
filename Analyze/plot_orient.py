import numpy as np
import matplotlib.pyplot as plt
import subprocess
import datetime

subprocess.call(['scp', 'sexybot:orient.npy', '.'])
orient_arr = np.load('orient.npy')

times = orient_arr[:, 0]
phi = orient_arr[:, 1]
phidot = orient_arr[:, 2]
phidotdot = orient_arr[:, 3]

fig, (ax1, ax2, ax3) = plt.subplots(3, figsize = (12,8), sharex = True)

ax1.plot(times, phi)
ax1.set_title('phi')
ax2.plot(times, phidot)
ax2.set_title('phidot')
ax3.plot(times, phidotdot)
ax3.set_title('phidotdot')
ax3.set_xlabel('time')
plt.show()

str_time = datetime.datetime.now().strftime("%d-%m_%H:%M")
np.save('datas/run_date-{}.npy'.format(str_time), orient_arr)
