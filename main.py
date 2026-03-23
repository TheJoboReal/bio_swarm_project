import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.animation import FuncAnimation

ARENA_SIDE_LENGTH = 20
NUMBER_OF_AGENTS = None
MAX_SPEED = 0.1
STEPS = 1200
VEL_MODE = 0
POS_MODE = 1

COHESION_THRESHOLD = 2
SEPERATION_THRESHOLD = 1.25
MIN_ALLIGNMENT_DISTANCE_THRESHOLD = 1.25


# Set up the output (1024 x 768):
fig = plt.figure(figsize=(10.24, 10.24), dpi=100)
ax = plt.axes(xlim=(0, ARENA_SIDE_LENGTH), ylim=(0, ARENA_SIDE_LENGTH))
(points,) = ax.plot(
    [],
    [],
    "bo",
    lw=0,
)


def init_agents():
    global x, y, vx, vy, x0, y0

    x = np.random.uniform(0, ARENA_SIDE_LENGTH, NUMBER_OF_AGENTS)
    y = np.random.uniform(0, ARENA_SIDE_LENGTH, NUMBER_OF_AGENTS)

    x0 = np.copy(x)
    y0 = np.copy(y)

    vx = np.random.uniform(-MAX_SPEED, MAX_SPEED, NUMBER_OF_AGENTS)
    vy = np.random.uniform(-MAX_SPEED, MAX_SPEED, NUMBER_OF_AGENTS)


def enforce_max_speed_limit(vx_agent, vy_agent):
    # Woop woop it's the sound of the police
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
# kilde: https://keyirobot.com/blogs/buying-guide/exploring-swarm-robotics-programming-multiple-simple-agents
# Kilde2: https://medium.com/better-programming/boids-simulating-birds-flock-behavior-in-python-9fff99375118
def cohesion(i, cohesion_threshold):  # tiltrækning til naboer
    # For every robot/agent/boid (find et navn) we need to find the average x and y position of its neighbors
    avg_x = 0
    avg_y = 0
    denominator_in_avg_calculation = 0.0

    # Loop through the robots and find avg
    for j in range(NUMBER_OF_AGENTS):
        if i != j:  # ensure different neighbor
            # Check that distnace to neighbor j is within threshold
            distance = distance_between_agents(i, j)
            if distance < cohesion_threshold:
                avg_x += x[j]  # ligger nabos position til avg_x
                avg_y += y[j]  # og for y også
                denominator_in_avg_calculation += (
                    1  # skal bruges til at dividere med for at få gennemsnit
                )

    # Find average by dividing by number of neigbors
    if denominator_in_avg_calculation > 0:
        avg_x = avg_x / denominator_in_avg_calculation
        avg_y = avg_y / denominator_in_avg_calculation
        return avg_x - x[i], avg_y - y[i]

    # Does nothing if no neighbor
    return 0.0, 0.0


#################### SEPARATION ####################
# kilde: https://keyirobot.com/blogs/buying-guide/exploring-swarm-robotics-programming-multiple-simple-agents
def separation(i, min_seperation_distance_threshold):
    # Which direction the agents will move according to seperation
    sx = 0.0
    sy = 0.0

    # Loop through nearby (given by threshold) neighbors
    for j in range(NUMBER_OF_AGENTS):
        if i != j:  # ensure different neighbor
            # Check that distnace to neighbor j is within threshold
            distance = distance_between_agents(i, j)
            if distance < min_seperation_distance_threshold:
                # Calc difference betweeen agent i and -j
                dx = x[j] - x[i]
                dy = y[j] - y[i]

                sx -= dx  # minus ensures agent moves away from other agents -> repels neighbors
                sy -= dy

    return sx, sy


#################### ALIGNMENT ####################
# kilde: https://keyirobot.com/blogs/buying-guide/exploring-swarm-robotics-programming-multiple-simple-agents
def alignment_Velocity_Based(i, min_alligment_distance_threshold):
    # variables for avg speed
    avg_vx = 0.0
    avg_vy = 0.0
    denominator_in_avg_calc = 0.0

    #  Find average speed of neighbors
    for j in range(NUMBER_OF_AGENTS):
        if j != i:  # ensure different neighbor
            # Check that distnace to neighbor j is within threshold
            distance = distance_between_agents(i, j)
            if distance < min_alligment_distance_threshold:
                avg_vx += vx[j]
                avg_vy += vy[j]
                denominator_in_avg_calc += 1

    # Find average by dividing by number of neigbors within given threshold
    if denominator_in_avg_calc > 0:
        avg_vx = avg_vx / denominator_in_avg_calc
        avg_vy = avg_vy / denominator_in_avg_calc

        # Fra kilde: Speed += allignment - current speed
        allignment_output_vx = avg_vx - vx[i]
        allignment_output_vy = avg_vy - vy[i]
        return allignment_output_vx, allignment_output_vy

    return 0.0, 0.0


#################### ALIGNMENT Position ####################
def alignment_Position_Based(i, min_alignment_distance_threshold, t):
    sum_dx = 0.0
    sum_dy = 0.0
    neighbor_count = 0

    for j in range(NUMBER_OF_AGENTS):
        if j != i:
            distance = distance_between_agents(i, j)

            if distance < min_alignment_distance_threshold:
                # Last relative position
                rel_x_now = x[j] - x[i]
                rel_y_now = y[j] - y[i]

                # Initial relative position
                rel_x_0 = x0[j] - x0[i]
                rel_y_0 = y0[j] - y0[i]

                # sum of changes
                sum_dx += rel_x_now - rel_x_0
                sum_dy += rel_y_now - rel_y_0

                neighbor_count += 1

    if neighbor_count > 0 and t > 0:
        # sum of change divided by number of neighbors
        avg_dx = sum_dx / neighbor_count
        avg_dy = sum_dy / neighbor_count

        # Approx relative velocity by the long-term change in relative position
        alignment_output_vx = avg_dx / t
        alignment_output_vy = avg_dy / t

        return alignment_output_vx, alignment_output_vy

    return 0.0, 0.0


def update(vx, vy, t):
    # Variables for updating speed if agents
    new_vx = np.copy(vx)
    new_vy = np.copy(vy)

    # weights for controlling cohesion, separation and allignment
    c_weight = 0.01  # 0.01
    s_weight = 0.05  # 0.05
    a_weight = 0.07  # 0.03

    # Update each robot individually
    for i in range(NUMBER_OF_AGENTS):
        cx, cy = cohesion(i, COHESION_THRESHOLD)  # print(cx, cy)
        sx, sy = separation(i, SEPERATION_THRESHOLD)
        ax, ay = alignment_Position_Based(i, MIN_ALLIGNMENT_DISTANCE_THRESHOLD, t)

        # Update speed
        new_vx[i] += c_weight * cx + s_weight * sx + a_weight * ax
        new_vy[i] += c_weight * cy + s_weight * sy + a_weight * ay

        # Enforce max speed
        new_vx[i], new_vy[i] = enforce_max_speed_limit(new_vx[i], new_vy[i])

    return new_vx, new_vy


# Make the environment toroidal
def wrap(z):
    return z % ARENA_SIDE_LENGTH


def init_terminal_input():
    global COHESION_THRESHOLD, SEPERATION_THRESHOLD
    global MIN_ALLIGNMENT_DISTANCE_THRESHOLD, NUMBER_OF_AGENTS

    try:
        NUMBER_OF_AGENTS = int(input("Enter number of agents (default 30): ") or 30)

        COHESION_THRESHOLD = float(input("Enter cohesion threshold (default 2): ") or 2)
        SEPERATION_THRESHOLD = float(
            input("Enter separation threshold (default 1.25): ") or 1.25
        )
        MIN_ALLIGNMENT_DISTANCE_THRESHOLD = float(
            input("Enter alignment distance threshold (default 1.25): ") or 1.25
        )
    except ValueError:
        print("Invalid input. Using default values.")
        NUMBER_OF_AGENTS = 30


def init():
    points.set_data([], [])
    return (points,)


def animate(i):
    global x, y, vx, vy
    # x = np.array(list(map(wrap, x + vx)))
    # y = np.array(list(map(wrap, y + vy)))

    # Update step
    vx, vy = update(vx, vy, i)

    x = wrap(x + vx)
    y = wrap(y + vy)

    points.set_data(x, y)
    print("Step ", i + 1, "/", STEPS, end="\r")

    return (points,)


init_terminal_input()
init_agents()
anim = FuncAnimation(fig, animate, init_func=init, frames=STEPS, interval=1, blit=True)

videowriter = animation.FFMpegWriter(fps=60)
anim.save("output.mp4", writer=videowriter)
