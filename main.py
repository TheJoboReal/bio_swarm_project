import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.animation import FuncAnimation
import cv2

ARENA_SIDE_LENGTH = 10
NUMBER_OF_ROBOTS = 50
STEPS = 1000
MAX_SPEED = 0.1

# Positions
x = np.random.uniform(low=0, high=ARENA_SIDE_LENGTH, size=(NUMBER_OF_ROBOTS,))
y = np.random.uniform(low=0, high=ARENA_SIDE_LENGTH, size=(NUMBER_OF_ROBOTS,))

# Velocities
vx = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_ROBOTS,))
vy = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_ROBOTS,))

# Set up the output (1024 x 768):
fig = plt.figure(figsize=(10.24, 7.68), dpi=100)
ax = plt.axes(xlim=(0, ARENA_SIDE_LENGTH), ylim=(0, ARENA_SIDE_LENGTH))
(points,) = ax.plot(
    [],
    [],
    "bo",
    lw=0,
)


# Make the environment toroidal
def wrap(z):
    return z % ARENA_SIDE_LENGTH


def init():
    points.set_data([], [])
    return (points,)


def animate(i):
    global x, y, vx, vy
    x = np.array(list(map(wrap, x + vx)))
    y = np.array(list(map(wrap, y + vy)))

    points.set_data(x, y)
    print("Step ", i + 1, "/", STEPS, end="\r")

    return (points,)


anim = FuncAnimation(fig, animate, init_func=init, frames=STEPS, interval=1, blit=True)

videowriter = animation.FFMpegWriter(fps=60)
anim.save("output.mp4", writer=videowriter)
