import sys, tty, termios, time

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

import socket

host = socket.gethostbyname('sexybot.local') #gethostname()
port = 8000                   # The same port as used by the server
print(host)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))

while True:
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        time.sleep(.04)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    if ord(ch) == 3:
        sys.exit()

    s.sendall(bytes(ch, 'utf-8'))

