import numpy as np
import math
import argparse
import matplotlib.pyplot as plt
import cv2
from agent import *

DEFAULT_NUMBER_OF_AGENTS = 30
DEFAULT_STEPS = 100
DEFAULT_MAX_SPEED = 2

DEFAULT_EPOCHS = 1

# Window width and height
DEFAULT_HEIGHT = 700
DEFAULT_WIDTH = 700


def parse_args():
    parser = argparse.ArgumentParser(description="Boids Simulation")
    parser.add_argument("--agents", type=int, default=DEFAULT_NUMBER_OF_AGENTS, help="Number of agents in the simulation")
    parser.add_argument("--steps", type=int, default=DEFAULT_STEPS, help="Number of simulation steps")
    parser.add_argument("--max_speed", type=float, default=DEFAULT_MAX_SPEED, help="Maximum speed of agents")
    parser.add_argument("--epochs", type=int, default=DEFAULT_EPOCHS, help="Number of epochs")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="Window height")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Window width")
    return parser.parse_args()


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


def cohesion_separation(boid, flock, sensor_range, delta):
    # Get index of current boid
    i = boid.get_id()

    # Count number of neighbors
    neighbor_count = 0
    for j in range(len(flock)):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:  # "0 <" to avoid dividing by zero later
                neighbor_count += 1  # Count number of neighbors

    # Calculate control input for boid i
    control_input_x, control_input_y = 0.0, 0.0
    for j in range(len(flock)):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:
                # Calculate cohesion-seperation gain:
                cohesion_separation_gain = 1 - (delta * neighbor_count) / distance

                # Then calculate cohesion_separation control input from cohesion_separation / attraction_repulsive-term
                boid_pos_x, boid_pos_y = boid.get_position()
                neighbor_pos_x, neighbor_pos_y = flock[j].get_position()

                control_input_x += cohesion_separation_gain * (
                    neighbor_pos_x - boid_pos_x
                )
                control_input_y += cohesion_separation_gain * (
                    neighbor_pos_y - boid_pos_y
                )
    # print(control_input_x, " ", control_input_y)

    return control_input_x, control_input_y


def alignment_velocity_based(boid, flock, sensor_range):
    # Current boid id and velocity
    i = boid.get_id()
    boid_vx, boid_vy = boid.get_velocity()

    # allignment variables
    alignment_x, alignment_y = 0.0, 0.0
    for j in range(len(flock)):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:  # "0 <" to avoid dividing by zero later
                neighbor_vel_x, neighbor_vel_y = flock[j].get_velocity()
                alignment_x += neighbor_vel_x - boid_vx
                alignment_y += neighbor_vel_y - boid_vy

    return alignment_x, alignment_y


def alignment_position_based(boid, flock, sensor_range, t):
    # Current boid id
    i = boid.get_id()

    sum_dx = 0.0
    sum_dy = 0.0
    neighbor_count = 0

    for j in range(len(flock)):
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

                neighbor_count += 1

    if neighbor_count > 0 and t > 0:
        # Approx relative velocity by the long-term change in relative position
        alignment_output_vx = sum_dx / t
        alignment_output_vy = sum_dy / t

        # print(alignment_output_vx, " " , alignment_output_vy)

        #print(t)

        return alignment_output_vx, alignment_output_vy
    elif neighbor_count > 0 and t < 10:
        k = 10
        alignment_output_vx = sum_dx / k
        alignment_output_vy = sum_dy / k

        # print(alignment_output_vx, " " , alignment_output_vy)
        return alignment_output_vx, alignment_output_vy

    return 0.0, 0.0


dt = 0.1  # ???


def update(flock, t, gamma_t, MAX_SPEED):

    t += dt

    # Update each boid individually
    for i in range(len(flock)):
        boid_og = flock[i]  # original
        sensor_range = 60
        cs_x, cs_y = cohesion_separation(boid_og, flock, sensor_range, delta=7.5)
        #ax_vel_based, ay_vel_based = alignment_velocity_based(boid_og, flock, sensor_range)
        ax_pos_based, ay_pos_based  = alignment_position_based(boid_og, flock, sensor_range, t)

        # Update speed for boid i
        vx, vy = boid_og.get_velocity()
        vx += dt * (cs_x + ax_pos_based)
        vy += dt * (cs_y + ay_pos_based)

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

    #print(t)
    #print(gamma_value)


def main():
    args = parse_args()

    NUMBER_OF_AGENTS = args.agents
    STEPS = args.steps
    MAX_SPEED = args.max_speed
    HEIGHT = args.height
    WIDTH = args.width

    EPOCHS = args.epochs


    img = (np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255 )  # for at få en hvid baggrund
    imgclear = (np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255)  # for at få en hvid baggrund

    blueColor = (255, 50, 50)
    agentRadius = 3 # for drawing

    # Generate random positions
    x = np.random.uniform(0, HEIGHT, size=(NUMBER_OF_AGENTS))
    y = np.random.uniform(0, WIDTH, size=(NUMBER_OF_AGENTS))

    # Generate random velocities
    vx = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_AGENTS,))
    vy = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_AGENTS,))

    # Make the flock
    flock = []
    for i in range(NUMBER_OF_AGENTS):
        flock.append(Boids(i, x[i], y[i], vx[i], vy[i], HEIGHT, WIDTH))

    # gamma_t directional alignment values over time.
    gamma_t = []


    # Do the simulation
    for epoch in range(EPOCHS):
        # Time passed
        T = 0.1

        while (T < DEFAULT_STEPS):
            print(T)
            img = imgclear.copy()  # to clear the image

            update(flock, T, gamma_t, MAX_SPEED)

            for i in range(NUMBER_OF_AGENTS):
                xBoid, yBoid = flock[i].get_position()
                cv2.circle(img, (int(xBoid), int(yBoid)), agentRadius, blueColor, -1)

            cv2.imshow("Window", img)

            if cv2.waitKey(1) == ord("q"):
                break


        cv2.destroyAllWindows()

        plt.plot(gamma_t)
        plt.savefig("gamma_t_plot.png")


############ Main
main()
