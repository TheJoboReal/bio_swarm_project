import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.animation import FuncAnimation
import cv2

from agent import *

ARENA_SIDE_LENGTH = 20
NUMBER_OF_ROBOTS = 30
STEPS = 1200
MAX_SPEED = 2


# Window width and height
HEIGHT = 700
WIDTH = 700

### Initiate random robots with random positions and velocities.
# Positions
# x = np.random.uniform(low=0, high=ARENA_SIDE_LENGTH, size=(NUMBER_OF_ROBOTS,))
# y = np.random.uniform(low=0, high=ARENA_SIDE_LENGTH, size=(NUMBER_OF_ROBOTS,))
#
# # Velocities
# vx = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_ROBOTS,))
# vy = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_ROBOTS,))


def distance_between_agents(boid_og, neighbor):
    x_og, y_og = boid_og.get_position()
    x_nb, y_nb = neighbor.get_position()

    dist_x = x_nb - x_og
    dist_y = y_nb - y_og
    distance = np.sqrt(dist_x**2 + dist_y**2)
    # print("distance ", distance)
    return distance


#################### COHESION ####################
# kilde: https://keyirobot.com/blogs/buying-guide/exploring-swarm-robotics-programming-multiple-simple-agents
# Kilde2: https://medium.com/better-programming/boids-simulating-birds-flock-behavior-in-python-9fff99375118
def cohesion(
    boid_og, flock, min_distanceThreshold_for_cohesion
):  # tiltrækning til naboer
    # For every robot/agent/boid (find et navn) we need to find the average x and y position of its neighbors
    avg_x = 0
    avg_y = 0
    denominator_in_avg_calculation = 0.0

    # Loop through the robots and find avg
    for j in range(NUMBER_OF_ROBOTS):
        if boid_og.get_id() != j:  # ensure different neighbor
            neighbor = flock[j]
            # Check that distnace to neighbor j is within threshold
            distance = distance_between_agents(boid_og, neighbor)
            if distance < min_distanceThreshold_for_cohesion:
                x_nb, y_nb = neighbor.get_position()

                avg_x += x_nb  # ligger nabos position til avg_x
                avg_y += y_nb  # og for y også
                denominator_in_avg_calculation += (
                    1  # skal bruges til at dividere med for at få gennemsnit
                )

    # Find average by dividing by number of neigbors
    if denominator_in_avg_calculation > 0:
        x_og, y_og = boid_og.get_position()

        avg_x = avg_x / denominator_in_avg_calculation
        avg_y = avg_y / denominator_in_avg_calculation
        return avg_x - x_og, avg_y - y_og

    # Does nothing if no neighbor
    return 0.0, 0.0


#################### SEPARATION ####################
# kilde: https://keyirobot.com/blogs/buying-guide/exploring-swarm-robotics-programming-multiple-simple-agents
def separation(boid_og, flock, min_seperation_distance_threshold):
    # Which direction the agents will move according to seperation
    sx = 0.0
    sy = 0.0

    # Loop through nearby (given by threshold) neighbors
    for j in range(NUMBER_OF_ROBOTS):
        if boid_og.get_id() != j:  # ensure different neighbor
            neighbor = flock[j]
            # Check that distnace to neighbor j is within threshold
            distance = distance_between_agents(boid_og, neighbor)
            if distance < min_seperation_distance_threshold:
                # Calc difference betweeen agent i and -j
                x_og, y_og = boid_og.get_position()
                x_nb, y_nb = neighbor.get_position()

                dx = x_nb - x_og
                dy = y_nb - y_og

                sx -= dx  # minus ensures agent moves away from other agents -> repels neighbors
                sy -= dy

    return sx, sy



#################### ALIGNMENT ####################
# kilde: https://keyirobot.com/blogs/buying-guide/exploring-swarm-robotics-programming-multiple-simple-agents
def alignment_Velocity_Based(boid_og, flock, min_alligment_distance_threshold):
    # variables for avg speed
    avg_vx = 0.0
    avg_vy = 0.0
    denominator_in_avg_calc = 0.0

    #  Find average speed of neighbors
    for j in range(NUMBER_OF_ROBOTS):
        if boid_og.get_id() != j:  # ensure different neighbor
            neighbor = flock[j]
            # Check that distnace to neighbor j is within threshold
            distance = distance_between_agents(boid_og, neighbor)
            if distance < min_alligment_distance_threshold:
                vx, vy = neighbor.get_velocity()
                avg_vx += vx
                avg_vy += vy
                denominator_in_avg_calc += 1

    # Find average by dividing by number of neigbors within given threshold
    if denominator_in_avg_calc > 0:
        avg_vx = avg_vx / denominator_in_avg_calc
        avg_vy = avg_vy / denominator_in_avg_calc

        # Fra kilde: Speed += allignment - current speed

        vx, vy = boid_og.get_velocity()
        allignment_output_vx = avg_vx - vx
        allignment_output_vy = avg_vy - vy
        return allignment_output_vx, allignment_output_vy

    return 0.0, 0.0


def cohesion_separation(boid_og, sensor_range, delta):
    # Get index of current boid
    i = boid_og.get_id()

    # Count number of neighbors
    neighbor_count = 0
    for j in range(NUMBER_OF_ROBOTS):
        if i != j:
            distance = distance_between_agents(i, j)
            if distance < sensor_range:
                neighbor_count += 1

    # Calculate control input (/acceleration) for agent i
    control_input = 0.0
    for j in range(NUMBER_OF_ROBOTS):
        if i != j:
            cohesion_separation_gain = i - delta * neighbor_count / np.sqrt( boid_og.g )
            control

    return 0.0, 0.0

def update_two(flock):

    # weights for controlling cohesion, separation and allignment
    c_weight = 0.01  # 0.01
    s_weight = 0.05  # 0.05
    a_weight = 0.03  # 0.03

    dt = 0.1

    # Update each robot individually
    for i in range(NUMBER_OF_ROBOTS):
        boid_og = flock[i]  # original
        cx, cy = cohesion(boid_og, flock, 80)  # print(cx, cy)
        sx, sy = separation(boid_og, flock, 15)
        ax, ay = alignment_Velocity_Based(boid_og, flock, 50)

        # Update speed
        vx, vy = boid_og.get_velocity()

        vx += dt * (c_weight * cx + s_weight * sx + a_weight * ax)
        vy += dt * (c_weight * cy + s_weight * sy + a_weight * ay)

        vx *= 0.99  # tips og tricks fra chatten --> 'prevents runaway acceleration'
        vy *= 0.99

        speed = np.sqrt(vx**2 + vy**2)  # speed limiter
        if speed > MAX_SPEED:
            vx = (vx / speed) * MAX_SPEED
            vy = (vy / speed) * MAX_SPEED

        flock[i].update_velocity(vx, vy)
        flock[i].update_position()  # points.set_data(x, y)


def main():
    img = (
        np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
    )  # for at få en hvid baggrund
    imgclear = (
        np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
    )  # for at få en hvid baggrund
    # (image, position, radius, color(BGR), thickness -1 = filled)
    # cv2.circle(img, (250, 250), 5, (255, 50, 50), -1)

    blueColor = (255, 50, 50)
    agentRadius = 5

    x = np.random.randint(
        200, 300, size=(NUMBER_OF_ROBOTS)
    )  # todo: mangler at tjekke om de spawner oven i hinanden
    y = np.random.randint(200, 300, size=(NUMBER_OF_ROBOTS))

    # Velocities
    vx = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_ROBOTS,))
    vy = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_ROBOTS,))

    flock = []  # Make the flock
    for i in range(NUMBER_OF_ROBOTS):
        flock.append(Boids(i, x[i], y[i], vx[i], vy[i], HEIGHT, WIDTH))

    hold = False  # Flag to control the hold state

    while True:
        img = imgclear.copy()  # to clear the image

        update_two(flock)

        for i in range(NUMBER_OF_ROBOTS):
            xBoid, yBoid = flock[i].get_position()
            cv2.circle(
                img, (int(xBoid), int(yBoid)), agentRadius, blueColor, -1
            )  # nu har vi tegnet direkte på vores frame, så kan vi tjekke farve i de omkring liggende pixels

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
