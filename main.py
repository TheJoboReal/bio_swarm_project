import numpy as np
import math
import matplotlib.pyplot as plt
import cv2
from agent import *

NUMBER_OF_AGENTS = 30
STEPS = 100
MAX_SPEED = 2
MAX_INITIAL_SPEED = 1

# Window width and height
HEIGHT = 700
WIDTH = 700


# Calculate the gamma [-1,1] value that represents the drectional alignment. If gamma is approx 1, it indicates near-parallel velocities (strong alignment) and values near or below 0 indicat-ing misalignment. By normalizing direction, this metric isolates directional consensus from speed differences
def directional_alignment(boid, flock, neighbor):
    vx_i, vy_i = boid.get_velocity()
    norm_vi = math.sqrt(vx_i**2 + vy_i**2)

    if norm_vi == 0:
        return 0

    i = boid.get_id()

    sum_alignment = 0.0
    neighbor_count = 0

    for j in neighbor:
        if i == j:
            continue

        vx_j, vy_j = flock[j].get_velocity()
        norm_vj = math.sqrt(vx_j**2 + vy_j**2)

        if norm_vj == 0:
            continue

        dot = vx_i * vx_j + vy_i * vy_j
        sum_alignment += dot / (norm_vi * norm_vj)
        neighbor_count += 1

    if neighbor_count == 0:
        return 0

    return sum_alignment / neighbor_count


def distance_between_agents(boid_og, neighbor):

    x_og, y_og = boid_og.get_position()
    x_nb, y_nb = neighbor.get_position()

    dist_x = x_nb - x_og
    dist_y = y_nb - y_og

    distance = np.sqrt(dist_x**2 + dist_y**2)
    # print("distance ", distance)
    return distance




#################### Velocity based  ####################
def control_input_velocity_based(boid, flock, sensor_range, delta):
    # Get index of current boid
    i = boid.get_id()

    #### Count number of neighbors
    neighbor_count = 0
    for j in range(NUMBER_OF_AGENTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:  # "0 <" to avoid dividing by zero later
                neighbor_count += 1  # Count number of neighbors

    #### Calculate cohesion separation
    cohesion_separation_x, cohesion_separation_y = 0.0, 0.0
    for j in range(NUMBER_OF_AGENTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:
                # Calculate cohesion-seperation gain:
                cohesion_separation_gain = 1 - (delta * neighbor_count) / distance

                # Then calculate cohesion_separation control input from cohesion_separation
                boid_pos_x, boid_pos_y = boid.get_position()
                neighbor_pos_x, neighbor_pos_y = flock[j].get_position()

                cohesion_separation_x += cohesion_separation_gain * (neighbor_pos_x - boid_pos_x)
                cohesion_separation_y += cohesion_separation_gain * (neighbor_pos_y - boid_pos_y)

    #### Calculate alignment (velocity based!)
    #   Get current velocity
    boid_vx, boid_vy = boid.get_velocity()

    # Calculate allignment part
    alignment_x, alignment_y = 0.0, 0.0
    for j in range(NUMBER_OF_AGENTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range: # "0 <" to avoid dividing by zero later
                neighbor_vx, neighbor_vy = flock[j].get_velocity()
                alignment_x += neighbor_vx - boid_vx
                alignment_y += neighbor_vy - boid_vy


    #### Control input for boid i
    control_input_x = cohesion_separation_x + alignment_x
    control_input_y = cohesion_separation_y + alignment_y

    return control_input_x, control_input_y







#################### Position Based Without Threshold ####################
def control_input_position_based_NO_threshold(boid, flock, sensor_range, delta, t):
    # Get index of current boid
    i = boid.get_id()

    #### Count number of neighbors
    neighbor_count = 0
    for j in range(NUMBER_OF_AGENTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range: # "0 <" to avoid dividing by zero later
                neighbor_count += 1 #Count number of neighbors


    #### Calculate cohesion separation
    cohesion_separation_x, cohesion_separation_y = 0.0, 0.0
    for j in range(NUMBER_OF_AGENTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:
                # Calculate cohesion-seperation gain:
                cohesion_separation_gain = 1 - (delta * neighbor_count) / distance

                # Then calculate cohesion_separation control input from cohesion_separation / attraction_repulsive-term
                boid_pos_x, boid_pos_y = boid.get_position()
                neighbor_pos_x, neighbor_pos_y = flock[j].get_position()

                cohesion_separation_x += cohesion_separation_gain * (neighbor_pos_x - boid_pos_x)
                cohesion_separation_y += cohesion_separation_gain * (neighbor_pos_y - boid_pos_y)


    ### Calculate alignment (Position based!)
    sum_dx = 0.0
    sum_dy = 0.0

    for j in range(NUMBER_OF_AGENTS):
        if j != i:
            distance = distance_between_agents(boid, flock[j])

            if distance < sensor_range:
                # Get current positions
                boid_pos_x, boid_pos_y = boid.get_position()
                neighbor_pos_x, neighbor_pos_y = flock[j].get_position()
                # Last relative position
                rel_x_now = neighbor_pos_x - boid_pos_x
                rel_y_now = neighbor_pos_y - boid_pos_y

                # Get initial positions
                boid_pos_x_init, boid_pos_y_init = boid.get_initial_position()
                neighbor_pos_x_init, neighbor_pos_y_init = flock[
                    j
                ].get_initial_position()
                # Initial relative position
                rel_x_0 = neighbor_pos_x_init - boid_pos_x_init
                rel_y_0 = neighbor_pos_y_init - boid_pos_y_init

                # sum of changes
                sum_dx += rel_x_now - rel_x_0
                sum_dy += rel_y_now - rel_y_0

    alignment_x = 1/t * sum_dx
    alignment_y = 1/t * sum_dy


    # Control input for boid i
    control_input_x = cohesion_separation_x + alignment_x
    control_input_y = cohesion_separation_y + alignment_y

    return control_input_x, control_input_y





#################### Position Based With Threshold ####################
def control_input_position_based_with_threshold(boid, flock, sensor_range, delta, t, k):
    # Get index of current boid
    i = boid.get_id()

    #### Count number of neighbors
    neighbor_count = 0
    for j in range(NUMBER_OF_AGENTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range: # "0 <" to avoid dividing by zero later
                neighbor_count += 1 #Count number of neighbors


    ### Calculate time-dependt alignment gain phi
    psi = 0.0
    if 0 < t < 1/k:
        psi = neighbor_count / t
    elif t > 1/k:
        psi = k * neighbor_count


    #### Calculate cohesion separation
    cohesion_separation_x, cohesion_separation_y = 0.0, 0.0
    for j in range(NUMBER_OF_AGENTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:

                # Calculate cohesion-seperation gain:
                cohesion_separation_gain = 1 - (delta * neighbor_count) / distance

                # Then calculate cohesion_separation control input from cohesion_separation / attraction_repulsive-term
                boid_pos_x, boid_pos_y = boid.get_position()
                neighbor_pos_x, neighbor_pos_y = flock[j].get_position()

                cohesion_separation_x += (cohesion_separation_gain + psi) * (neighbor_pos_x - boid_pos_x)
                cohesion_separation_y += (cohesion_separation_gain + psi) * (neighbor_pos_y - boid_pos_y)


    ### Calculate alignment (Position based!)
    sum_dx = 0.0
    sum_dy = 0.0

    for j in range(NUMBER_OF_AGENTS):
        if j != i:
            distance = distance_between_agents(boid, flock[j])

            if distance < sensor_range:
                # Get current positions
                boid_pos_x, boid_pos_y = boid.get_position()
                neighbor_pos_x, neighbor_pos_y = flock[j].get_position()
                # Last relative position
                rel_x_now = neighbor_pos_x - boid_pos_x
                rel_y_now = neighbor_pos_y - boid_pos_y

                #Get initial positions
                boid_pos_x_init, boid_pos_y_init = boid.get_initial_position()
                neighbor_pos_x_init, neighbor_pos_y_init = flock[j].get_initial_position()
                # Initial relative position
                rel_x_0 = neighbor_pos_x_init - boid_pos_x_init
                rel_y_0 = neighbor_pos_y_init - boid_pos_y_init

                # sum of changes
                sum_dx += rel_x_now - rel_x_0
                sum_dy += rel_y_now - rel_y_0

    alignment_x = psi * sum_dx
    alignment_y = psi * sum_dy


    ### Control input for boid i
    control_input_x = cohesion_separation_x + alignment_x
    control_input_y = cohesion_separation_y + alignment_y

    return control_input_x, control_input_y


dt = 0.1  # ???


def update(flock, t, gamma_t):




def update(flock, t):
    
    #Update time step
    dt = 0.1
    t += dt

    # Update each boid individually
    for i in range(NUMBER_OF_AGENTS):
        boid_og = flock[i]  # original
        sensor_range = 60
        delta = 7.5
        u_x_vel_based, u_y_vel_based = control_input_velocity_based(boid_og, flock, sensor_range, delta)
        u_x_pos_based, u_y_pos_based = control_input_position_based_NO_threshold(boid_og, flock, sensor_range, delta, t)
        u_x_pos_based_threshold, u_y_pos_based_threshold = control_input_position_based_with_threshold(boid_og, flock, sensor_range, delta, t, k=0.1)


        # Update speed for boid i
        vx, vy = boid_og.get_velocity()
        vx += dt * u_x_vel_based
        vy += dt * u_y_vel_based

        # speed limiter
        speed = np.sqrt(vx**2 + vy**2)
        if speed > MAX_SPEED:
            vx = (vx / speed) * MAX_SPEED
            vy = (vy / speed) * MAX_SPEED

        # Execute update for boid i
        flock[i].update_velocity(vx, vy)
        flock[i].update_position()  # points.set_data(x, y)

    gamma_sum = 0.0
    for i in range(len(flock)):
        boid_i = flock[i]
        gamma_sum += directional_alignment(boid_i, flock, neighbor=range(len(flock)))

    gamma_value = gamma_sum / len(flock)
    gamma_t.append(gamma_value)

    print(t)
    print(gamma_value)


def main():
    img = (
        np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
    )  # for at få en hvid baggrund
    imgclear = (
        np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
    )  # for at få en hvid baggrund

    blueColor = (255, 50, 50)
    agentRadius = 2

    # Generate random positions
    x = np.random.randint(200, 300, size=(NUMBER_OF_AGENTS))
    y = np.random.randint(200, 300, size=(NUMBER_OF_AGENTS))

    # Generate random velocities
    vx = np.random.uniform(low=-MAX_INITIAL_SPEED, high=MAX_INITIAL_SPEED, size=(NUMBER_OF_AGENTS,))
    vy = np.random.uniform(low=-MAX_INITIAL_SPEED, high=MAX_INITIAL_SPEED, size=(NUMBER_OF_AGENTS,))

    # Make the flock
    flock = []
    for i in range(NUMBER_OF_AGENTS):
        flock.append(Boids(i, x[i], y[i], vx[i], vy[i], HEIGHT, WIDTH))

    # hold = False  # Flag to control the hold state
    # gamma_t directional alignment values over time.
    gamma_t = []

    # Time passed
    T = 0.1

    # Do the simulation
    while True:
        img = imgclear.copy()  # to clear the image

        update(flock, T, gamma_t)

        for i in range(NUMBER_OF_AGENTS):
            xBoid, yBoid = flock[i].get_position()
            cv2.circle(img, (int(xBoid), int(yBoid)), agentRadius, blueColor, -1)

        cv2.imshow("Window", img)

        if cv2.waitKey(1) == ord("q"):
            break

        # Hvis man vil steppe through, evt med et print i en funktion så man kan nå at stoppe op og se hvad der står.
        # key = cv2.waitKey(0)
        #
        # if key == ord('q'):
        #     break
        # elif key == ord('e'):
        #     hold = not hold
        #     if not hold:
        #         continue
        # elif hold:
        #     continue

    cv2.destroyAllWindows()

    plt.plot(gamma_t)
    plt.savefig("gamma_t_plot.png")


############ Main
main()
