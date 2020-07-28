import matplotlib.pyplot as plt
import numpy as np
import sys, tty, termios, time
import socket

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

host = socket.gethostbyname('sexybot.local') #gethostname()
port = 8000                   # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))

class Cursor(object):
    def __init__(self, ax):
        self.ax = ax
        self.lx = ax.axhline(color='k')  # the horiz line
        self.ly = ax.axvline(color='k')  # the vert line

        # text location in axes coords
        self.txt = ax.text(0.7, 0.9, '', transform=ax.transAxes)

    def mouse_move(self, event):
        if not event.inaxes:
            x, y = 0, 0
        else:
            x, y = event.xdata, event.ydata

        # update the line positions
        print(x, y)

        s.sendall(bytes('{:.02f},{:.02f}'.format(-x,y), 'utf-8'))
        self.lx.set_ydata(y)
        self.ly.set_xdata(x)

        self.txt.set_text('x=%1.2f, y=%1.2f' % (x, y))
        self.ax.figure.canvas.draw()



#t = np.arange(-10.0, 10.0, 0.1)
#s = np.sin(2 * 2 * np.pi * t)

fig, ax = plt.subplots()
ax.set_xlim([-10,10])
ax.set_ylim([-.02,.02])
cursor = Cursor(ax)
fig.canvas.mpl_connect('motion_notify_event', cursor.mouse_move)

plt.show()
