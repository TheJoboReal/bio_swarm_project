import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.animation import FuncAnimation

ARENA_SIDE_LENGTH = 20
NUMBER_OF_ROBOTS = 30
STEPS = 1#1200
FPS = 60
MAX_SPEED = 0.1


### Initiate random robots with random positions and velocities.
# Positions
# x = np.random.uniform(low=0, high=ARENA_SIDE_LENGTH, size=(NUMBER_OF_ROBOTS,))
# y = np.random.uniform(low=0, high=ARENA_SIDE_LENGTH, size=(NUMBER_OF_ROBOTS,))
x = np.random.uniform(low=ARENA_SIDE_LENGTH/4, high=3*ARENA_SIDE_LENGTH/4, size=(NUMBER_OF_ROBOTS,)) #Initiate agents in the "middle" of arena
y = np.random.uniform(low=ARENA_SIDE_LENGTH/4, high=3*ARENA_SIDE_LENGTH/4, size=(NUMBER_OF_ROBOTS,)) #Initiate agents in the "middle" of arena

#Remember initial position for each agent
initial_position_x = x
initial_position_y = y

# Velocities
vx = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_ROBOTS,))
vy = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_ROBOTS,))

# Set up the output (1024 x 768):
fig = plt.figure(figsize=(10.24, 10.24), dpi=100)
ax = plt.axes(xlim=(0, ARENA_SIDE_LENGTH), ylim=(0, ARENA_SIDE_LENGTH))
(points,) = ax.plot(    
    [],
    [],
    "bo",
    lw=0,
)


def enforce_max_speed_limit(vx_agent, vy_agent):
    #Woop woop it's the sound of the police
    speed = np.sqrt(vx_agent**2 + vy_agent**2)

    if speed > MAX_SPEED:
        vx_agent = vx_agent / speed * MAX_SPEED
        vy_agent = vy_agent / speed * MAX_SPEED
    return vx_agent, vy_agent


def distance_between_agents(i, j):
    dist_x = x[j] - x[i]
    dist_y = y[j] - y[i]
    distance = np.sqrt(dist_x**2 + dist_y**2)
    return distance




def cohesion_separation(i, sensor_range, delta):
    #Count number of neighbors
    neighbor_count = 0
    for j in range(NUMBER_OF_ROBOTS):
        if i != j:

            distance = distance_between_agents(i,j)
            if distance < sensor_range:
                neighbor_count += 1

    #Calculate control input (/acceleration) for agent i
    control_input = 0.0
    for j in range(NUMBER_OF_ROBOTS):
        if i != j:
            cohesion_separation_gain = i - delta * neighbor_count / np.sqrt(x[i]**2 + y[i]**2)
            control

    return 0.0, 0.0


def update(vx, vy):
    #Variables for updating speed if agents
    new_vx = np.copy(vx)
    new_vy = np.copy(vy)

    #weights for controlling cohesion, separation and allignment
    cohesion_weight = 0.01 #0.01
    separation_weight = 0.05 #0.05
    alignment_weight = 0.07 #0.03

    # Update for each robot individually
    for i in range(NUMBER_OF_ROBOTS):
        cohesion_vx, cohesion_vy = cohesion_separation(i, sensor_range=4.0, delta=1.5)



        #### Update speed...
        #... from cohesion
        new_vx[i] += cohesion_weight * cohesion_vx
        new_vy[i] += cohesion_weight * cohesion_vy




        #Enforce max speed
        new_vx[i], new_vy[i] = enforce_max_speed_limit(new_vx[i], new_vy[i])

    return new_vx, new_vy


# Make the environment toroidal
def wrap(z):
    return z % ARENA_SIDE_LENGTH


def init():
    points.set_data([], []) 
    return (points,)


def animate(i):
    global x, y, vx, vy
    # x = np.array(list(map(wrap, x + vx)))
    # y = np.array(list(map(wrap, y + vy)))

    #Update step
    vx, vy = update(vx, vy)

    #Update position
    dt = 1/FPS    
    x = wrap(x + vx)
    y = wrap(y + vy)

    points.set_data(x, y)
    print("Step ", i + 1, "/", STEPS, end="\r")

    return (points,)


anim = FuncAnimation(fig, animate, init_func=init, frames=STEPS, interval=1, blit=True)

videowriter = animation.FFMpegWriter(fps=FPS)
anim.save("output.mp4", writer=videowriter)
