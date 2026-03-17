import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.animation import FuncAnimation

ARENA_SIDE_LENGTH = 20
NUMBER_OF_ROBOTS = 30
STEPS = 1200
MAX_SPEED = 0.1


### Initiate random robots with random positions and velocities.
# Positions
x = np.random.uniform(low=0, high=ARENA_SIDE_LENGTH, size=(NUMBER_OF_ROBOTS,))
y = np.random.uniform(low=0, high=ARENA_SIDE_LENGTH, size=(NUMBER_OF_ROBOTS,))

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


#################### COHESION ####################
#kilde: https://keyirobot.com/blogs/buying-guide/exploring-swarm-robotics-programming-multiple-simple-agents
#Kilde2: https://medium.com/better-programming/boids-simulating-birds-flock-behavior-in-python-9fff99375118
def cohesion(i, min_distanceThreshold_for_cohesion_bedreNavnErEnGodIde): #tiltrækning til naboer
    # For every robot/agent/boid (find et navn) we need to find the average x and y position of its neighbors
    avg_x = 0
    avg_y = 0
    denominator_in_avg_calculation = 0.0

    # Loop through the robots and find avg
    for j in range(NUMBER_OF_ROBOTS):
        if i != j: #ensure different neighbor

            #Check that distnace to neighbor j is within threshold
            distance = distance_between_agents(i, j)
            if distance < min_distanceThreshold_for_cohesion_bedreNavnErEnGodIde:
                avg_x += x[j] # ligger nabos position til avg_x
                avg_y += y[j] # og for y også
                denominator_in_avg_calculation += 1 #skal bruges til at dividere med for at få gennemsnit

    # Find average by dividing by number of neigbors
    if denominator_in_avg_calculation > 0:
        avg_x = avg_x / denominator_in_avg_calculation
        avg_y = avg_y / denominator_in_avg_calculation
        return avg_x - x[i], avg_y - y[i] 

    #Does nothing if no neighbor
    return 0.0, 0.0



#################### SEPARATION ####################
#kilde: https://keyirobot.com/blogs/buying-guide/exploring-swarm-robotics-programming-multiple-simple-agents
def separation(i, min_seperation_distance_threshold):
    # Which direction the agents will move according to seperation
    sx = 0.0
    sy = 0.0
    
    #Loop through nearby (given by threshold) neighbors
    for j in range(NUMBER_OF_ROBOTS):
        if i != j:#ensure different neighbor

            #Check that distnace to neighbor j is within threshold
            distance = distance_between_agents(i, j)
            if distance < min_seperation_distance_threshold:
                #Calc difference betweeen agent i and -j
                dx = x[j] - x[i]
                dy = y[j] - y[i]

                sx -= dx #minus ensures agent moves away from other agents -> repels neighbors
                sy -= dy
                
    return sx, sy


#################### ALIGNMENT ####################
#kilde: https://keyirobot.com/blogs/buying-guide/exploring-swarm-robotics-programming-multiple-simple-agents
def alignment_Velocity_Based(i, min_alligment_distance_threshold):
    #variables for avg speed
    avg_vx = 0.0
    avg_vy = 0.0
    denominator_in_avg_calc = 0.0

    #  Find average speed of neighbors
    for j in range(NUMBER_OF_ROBOTS):
        if j != i: #ensure different neighbor
            
            #Check that distnace to neighbor j is within threshold
            distance = distance_between_agents(i, j)
            if distance < min_alligment_distance_threshold:
                avg_vx += vx[j]
                avg_vy += vy[j]
                denominator_in_avg_calc += 1
    
    # Find average by dividing by number of neigbors within given threshold
    if denominator_in_avg_calc > 0:
        avg_vx = avg_vx / denominator_in_avg_calc
        avg_vy = avg_vy / denominator_in_avg_calc

        #Fra kilde: Speed += allignment - current speed
        allignment_output_vx = avg_vx - vx[i]
        allignment_output_vy = avg_vy - vy[i]
        return allignment_output_vx, allignment_output_vy

    return 0.0, 0.0




def update(vx, vy):
    #Variables for updating speed if agents
    new_vx = np.copy(vx)
    new_vy = np.copy(vy)

    #weights for controlling cohesion, separation and allignment
    c_weight = 0.01 #0.01
    s_weight = 0.05 #0.05
    a_weight = 0.07 #0.03

    # Update each robot individually
    for i in range(NUMBER_OF_ROBOTS):
        cx, cy = cohesion(i, 1.5); #print(cx, cy)
        sx, sy = separation(i, 1.0)
        ax, ay = alignment_Velocity_Based(i, 1.25)


        #Update speed
        new_vx[i] += c_weight*cx + s_weight*sx + a_weight*ax
        new_vy[i] += c_weight*cy + s_weight*sy + a_weight*ay

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

    
    x = wrap(x + vx)
    y = wrap(y + vy)

    points.set_data(x, y)
    print("Step ", i + 1, "/", STEPS, end="\r")

    return (points,)


anim = FuncAnimation(fig, animate, init_func=init, frames=STEPS, interval=1, blit=True)

videowriter = animation.FFMpegWriter(fps=60)
anim.save("output.mp4", writer=videowriter)
