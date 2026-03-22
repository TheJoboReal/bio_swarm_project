import numpy as np
import cv2

from agent import *

ARENA_SIDE_LENGTH = 20
NUMBER_OF_ROBOTS = 2 #30
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
    for j in range(NUMBER_OF_ROBOTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            print("cs n dist: ", distance)
            if 0 < distance < sensor_range: # "0 <" to avoid dividing by zero later
                neighbor_count += 1 #Count number of neighbors

    # Calculate control input for boid i
    control_input_x, control_input_y = 0.0, 0.0
    for j in range(NUMBER_OF_ROBOTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            print("cs ci dist: ", distance)
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
    boid_vel_x, boid_vel_y = boid.get_velocity()

    #allignment variables
    alignment_x, alignment_y = 0.0, 0.0
    for j in range(NUMBER_OF_ROBOTS):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            print("a dist: ", distance)
            if 0 < distance < sensor_range: # "0 <" to avoid dividing by zero later
                neighbor_vel_x, neighbor_vel_y = flock[j].get_velocity()
                alignment_x += neighbor_vel_x - boid_vel_x
                alignment_y += neighbor_vel_y - boid_vel_y
    
    return alignment_x, alignment_y


def update(flock):

    dt = 0.1 #???

    # Update each boid individually
    for i in range(NUMBER_OF_ROBOTS):
        boid_og = flock[i]  # original
        sensor_range = 60
        cs_x, cs_y = cohesion_separation(boid_og, flock, sensor_range, delta=7.5)
        ax, ay = alignment_velocity_based(boid_og, flock, sensor_range)

        # Update speed for boid i
        vx, vy = boid_og.get_velocity()

        vx += dt * (cs_x + ax)
        vy += dt * (cs_y + ay)

        speed = np.sqrt(vx**2 + vy**2)  # speed limiter
        if speed > MAX_SPEED:
            vx = (vx / speed) * MAX_SPEED
            vy = (vy / speed) * MAX_SPEED

        flock[i].update_velocity(vx, vy)
        flock[i].update_position()  # points.set_data(x, y)

    p1x, p1y = flock[0].get_position()
    p2x, p2y = flock[1].get_position()
    print("\npos1: ", round(p1x,2), " ", round(p1y,2))
    print("pos2: ", round(p2x,2), " ", round(p2y,2))
    print("dist: ", round(distance_between_agents(flock[0], flock[1]),2))


def main():
    img = (np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255)  # for at få en hvid baggrund
    imgclear = (np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255)  # for at få en hvid baggrund
    # (image, position, radius, color(BGR), thickness -1 = filled)
    # cv2.circle(img, (250, 250), 5, (255, 50, 50), -1)

    blueColor = (255, 50, 50)
    agentRadius = 3

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

        update(flock)

        for i in range(NUMBER_OF_ROBOTS):
            xBoid, yBoid = flock[i].get_position()
            cv2.circle(img, (int(xBoid), int(yBoid)), agentRadius, blueColor, -1)

        cv2.imshow("Window", img)

        while(1):
            if cv2.waitKey(1) == ord("h"):
                break

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
