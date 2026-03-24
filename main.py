import numpy as np
import cv2
from agent import *

NUMBER_OF_AGENTS = 30
STEPS = 100
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
    #print(control_input_x, " ", control_input_y)
    
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




def alignment_position_based(boid, flock, sensor_range, t):
    #Current boid id
    i = boid.get_id()


    sum_dx = 0.0
    sum_dy = 0.0
    neighbor_count = 0

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

                neighbor_count += 1

    if neighbor_count > 0 and t > 0:

        # Approx relative velocity by the long-term change in relative position
        alignment_output_vx = sum_dx / t
        alignment_output_vy = sum_dy / t

        #print(alignment_output_vx, " " , alignment_output_vy)

        print(t)

        return alignment_output_vx, alignment_output_vy
    elif neighbor_count > 0 and t < 10:
        k = 10
        alignment_output_vx = sum_dx / k
        alignment_output_vy = sum_dy / k

        #print(alignment_output_vx, " " , alignment_output_vy)
        return alignment_output_vx, alignment_output_vy
        

    return 0.0, 0.0







dt = 0.1 #???

def update(flock, t):
    
    t += dt 

    # Update each boid individually
    for i in range(NUMBER_OF_AGENTS):
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

        #Execute update for boid i
        flock[i].update_velocity(vx, vy)
        flock[i].update_position()  # points.set_data(x, y)

    return t

def main():
    img = (np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255)  # for at få en hvid baggrund
    imgclear = (np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255)  # for at få en hvid baggrund

    blueColor = (255, 50, 50)
    agentRadius = 3

    # Generate random positions
    x = np.random.randint(200, 300, size=(NUMBER_OF_AGENTS))
    y = np.random.randint(200, 300, size=(NUMBER_OF_AGENTS))

    # Generate random velocities
    vx = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_AGENTS,))
    vy = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_AGENTS,))

    # Make the flock
    flock = []
    for i in range(NUMBER_OF_AGENTS):
        flock.append(Boids(i, x[i], y[i], vx[i], vy[i], HEIGHT, WIDTH))

    # Time variable
    timeIterator = 0

    # Do the simulation
    while True:
        img = imgclear.copy()  # to clear the image

        timeIterator += update(flock, timeIterator)

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


############ Main
main()
