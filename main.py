import numpy as np
import cv2
from agent import *

ARENA_SIDE_LENGTH = 20
NUMBER_OF_AGENTS = 30
STEPS = 1200
MAX_SPEED = 2

# Window width and height
HEIGHT = 700
WIDTH = 700


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
    for j in range(NUMBER_OF_AGENTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range: # "0 <" to avoid dividing by zero later
                neighbor_count += 1 #Count number of neighbors

    # Calculate control input for boid i
    control_input_x, control_input_y = 0.0, 0.0
    for j in range(NUMBER_OF_AGENTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:

                # Calculate cohesion-seperation gain:
                cohesion_separation_gain = 1 - (delta * neighbor_count) / distance

                # Then calculate cohesion_separation control input from cohesion_separation / attraction_repulsive-term
                boid_pos_x, boid_pos_y = boid.get_position()
                neighbor_pos_x, neighbor_pos_y = flock[j].get_position()

                control_input_x += cohesion_separation_gain * (neighbor_pos_x - boid_pos_x)
                control_input_y += cohesion_separation_gain * (neighbor_pos_y - boid_pos_y)

    return control_input_x, control_input_y


def alignment_velocity_based(boid, flock, sensor_range):
    #Current boid id and velocity
    i = boid.get_id()
    boid_vx, boid_vy = boid.get_velocity()

    #allignment variables
    alignment_x, alignment_y = 0.0, 0.0
    for j in range(NUMBER_OF_AGENTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range: # "0 <" to avoid dividing by zero later
                neighbor_vel_x, neighbor_vel_y = flock[j].get_velocity()
                alignment_x += neighbor_vel_x - boid_vx
                alignment_y += neighbor_vel_y - boid_vy
    
    return alignment_x, alignment_y


def alignment_position_based(boid, flock, sensor_range, T):
    #Current boid's id
    i = boid.get_id()
    #Current boid's initial position
    boid_i_initial_x, boid_i_initial_y = boid.get_initial_position()

    #allignment variables
    sum_dx, sum_dy = 0.0, 0.0
    for j in range(NUMBER_OF_AGENTS):
        if j != i:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range: 
                #Boid_i's current position
                boid_i_x, boid_i_y = boid.get_position()
                #Neighbors j's current position
                neighbor_j_x, neighbor_j_y = flock[j].get_position()
                # Last relative position
                rel_x_now = neighbor_j_x - boid_i_x
                rel_y_now = neighbor_j_y - boid_i_y

                # Neighbor boid's initial position
                neighbor_j_initial_x, neighbor_j_initial_y = flock[j].get_initial_position()
                # Initial relative position
                rel_x_0 = neighbor_j_initial_x - boid_i_initial_x
                rel_y_0 = neighbor_j_initial_y - boid_i_initial_y

                # Sum of changes
                sum_dx += rel_x_now - rel_x_0
                sum_dy += rel_y_now - rel_y_0


    alignment_x = sum_dx / T
    alignment_y = sum_dy / T
    return alignment_x, alignment_y


def update(flock, T):

    #Time steps
    dt = 0.1

    # Update each boid individually
    for i in range(NUMBER_OF_AGENTS):
        boid_og = flock[i]  # original
        sensor_range = 60
        cs_x, cs_y = cohesion_separation(boid_og, flock, sensor_range, delta=7.5)
        ax_vel_based, ay_vel_based = alignment_velocity_based(boid_og, flock, sensor_range)
        ax_pos_based, ay_pos_based = alignment_position_based(boid_og, flock, sensor_range, T)

        # Update speed for boid i
        vx, vy = boid_og.get_velocity()
        vx += dt * (cs_x + ax_pos_based)
        vy += dt * (cs_y + ay_pos_based)

        # speed limiter
        speed = np.sqrt(vx**2 + vy**2)  
        if speed > MAX_SPEED:
            vx = (vx / speed) * MAX_SPEED
            vy = (vy / speed) * MAX_SPEED

        #Execute update for boid i
        flock[i].update_velocity(vx, vy)
        flock[i].update_position()  # points.set_data(x, y)

    print(T)


def main():
    img = (np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255)  # for at få en hvid baggrund
    imgclear = (np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255)  # for at få en hvid baggrund

    blueColor = (255, 50, 50)
    agentRadius = 3

    x = np.random.randint(
        200, 300, size=(NUMBER_OF_AGENTS)
    )  # todo: mangler at tjekke om de spawner oven i hinanden
    y = np.random.randint(200, 300, size=(NUMBER_OF_AGENTS))

    # Velocities
    vx = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_AGENTS,))
    vy = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_AGENTS,))

    flock = []  # Make the flock
    for i in range(NUMBER_OF_AGENTS):
        flock.append(Boids(i, x[i], y[i], vx[i], vy[i], HEIGHT, WIDTH))

    # hold = False  # Flag to control the hold state

    #Time passed
    T = 0.1

    while True:
        img = imgclear.copy()  # to clear the image

        update(flock, T)

        for i in range(NUMBER_OF_AGENTS):
            xBoid, yBoid = flock[i].get_position()
            cv2.circle(img, (int(xBoid), int(yBoid)), agentRadius, blueColor, -1)

        T += 0.1

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


############ Main
main()
