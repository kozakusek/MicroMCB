import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from threading import Thread, Lock
from ctypes import c_int8

fig, ax = plt.subplots()

SPEED = 0.01
mutex = Lock()
X = 0
Y = 0
U = 0
V = 0

def get_input():
    global X, Y, U, V, mutex
    cnt = 0
    with open("/dev/rfcomm1", "rb", buffering=0) as rfcomm:
        while True:
            start = rfcomm.read(1)[0]
            if start != 1:
                continue
            with mutex:
                U = c_int8(rfcomm.read(1)[0]).value / 127
                V = c_int8(rfcomm.read(1)[0]).value / 127
                X += U * SPEED
                Y += V * SPEED
            

modifier = Thread(target=get_input)
modifier.start()

def get_arrow(_theta):
    return X, Y, U, V

quiver = ax.quiver(*get_arrow(0))

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)

def update(theta):
    global quiver, mutex
    with mutex:
        quiver.remove()
        quiver = ax.quiver(*get_arrow(theta), angles="xy", scale_units="xy", scale=1)
        ax.set_xlim(X - 1, X + 1)
        ax.set_ylim(Y - 1, Y + 1)


ani = FuncAnimation(fig, update, interval=1)
plt.show()