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


# uv run main.py --agents 50 --steps 40 --max_speed 1.5 --epochs 2 --height 700 --width 700 --mode position_threshold
# uv run main.py --agents 50 --steps 100 --max_speed 2 --epochs 3 --height 700 --width 700
def parse_args():
    parser = argparse.ArgumentParser(description="Boids Simulation")
    parser.add_argument(
        "--agents",
        type=int,
        default=DEFAULT_NUMBER_OF_AGENTS,
        help="Number of agents in the simulation",
    )
    parser.add_argument(
        "--steps", type=int, default=DEFAULT_STEPS, help="Number of simulation steps"
    )
    parser.add_argument(
        "--max_speed",
        type=float,
        default=DEFAULT_MAX_SPEED,
        help="Maximum speed of agents",
    )
    parser.add_argument(
        "--epochs", type=int, default=DEFAULT_EPOCHS, help="Number of epochs"
    )
    parser.add_argument(
        "--height", type=int, default=DEFAULT_HEIGHT, help="Window height"
    )
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Window width")
    parser.add_argument(
        "--mode",
        type=str,
        default="velocity",
        choices=["velocity", "position", "position_threshold"],
        help="Simulation mode: 'velocity', 'position', or 'position_threshold'",
    )
    return parser.parse_args()


def exponential_smoothing(data, alpha=0.1):
    smoothed = [data[0]]  # first value as seed
    for x in data[1:]:
        smoothed.append(alpha * x + (1 - alpha) * smoothed[-1])
    return smoothed


# Calculate the gamma [-1,1] value that represents the drectional alignment. If gamma is approx 1, it indicates near-parallel velocities (strong alignment) and values near or below 0 indicat-ing misalignment. By normalizing direction, this metric isolates directional consensus from speed differences
def directional_alignment_metric(boid, flock, neighbor):
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


#################### Distance functions  ####################
def distance_between_agents(
    boid_og, neighbor
):  # Normal distance function that takes care of wrap
    x_og, y_og = boid_og.get_position()
    x_nb, y_nb = neighbor.get_position()

    dx = x_nb - x_og
    dy = y_nb - y_og

    # Wrap around in x
    if dx > boid_og.width / 2:
        dx -= boid_og.width
    elif dx < -boid_og.width / 2:
        dx += boid_og.width

    # Wrap around in y
    if dy > boid_og.height / 2:
        dy -= boid_og.height
    elif dy < -boid_og.height / 2:
        dy += boid_og.height

    return np.sqrt(dx**2 + dy**2)


def relative_position(
    boid_og, neighbor
):  # Fucntion for calculating relative distance (used in all three "modes")
    x_og, y_og = boid_og.get_position()
    x_nb, y_nb = neighbor.get_position()

    dx = x_nb - x_og
    dy = y_nb - y_og

    if dx > boid_og.width / 2:
        dx -= boid_og.width
    elif dx < -boid_og.width / 2:
        dx += boid_og.width

    if dy > boid_og.height / 2:
        dy -= boid_og.height
    elif dy < -boid_og.height / 2:
        dy += boid_og.height

    return dx, dy


#################### Velocity based  ####################
def control_input_velocity_based(boid, flock, sensor_range, delta):
    # Get index of current boid
    i = boid.get_id()

    #### Count number of neighbors
    neighbor_count = 0
    for j in range(len(flock)):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:  # "0 <" to avoid dividing by zero later
                neighbor_count += 1  # Count number of neighbors

    #### Calculate cohesion separation
    cohesion_separation_x, cohesion_separation_y = 0.0, 0.0
    for j in range(len(flock)):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:
                # Calculate cohesion-seperation gain:
                cohesion_separation_gain_psi = 1 - (delta * neighbor_count) / distance

                # Then calculate cohesion_separation control input from cohesion_separation
                # boid_pos_x, boid_pos_y = boid.get_position()
                # neighbor_pos_x, neighbor_pos_y = flock[j].get_position()
                dx, dy = relative_position(boid, flock[j])

                cohesion_separation_x += (
                    cohesion_separation_gain_psi * dx
                )  # (neighbor_pos_x - boid_pos_x)
                cohesion_separation_y += (
                    cohesion_separation_gain_psi * dy
                )  # (neighbor_pos_y - boid_pos_y)

    #### Calculate alignment (velocity based!)
    #   Get current velocity
    boid_vx, boid_vy = boid.get_velocity()

    # Calculate allignment part
    alignment_x, alignment_y = 0.0, 0.0
    for j in range(len(flock)):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:  # "0 <" to avoid dividing by zero later
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
    for j in range(len(flock)):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:  # "0 <" to avoid dividing by zero later
                neighbor_count += 1  # Count number of neighbors

    #### Calculate cohesion separation
    cohesion_separation_x, cohesion_separation_y = 0.0, 0.0
    for j in range(len(flock)):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:
                # Calculate cohesion-seperation gain:
                cohesion_separation_gain_psi = 1 - (delta * neighbor_count) / distance

                # Then calculate cohesion_separation control input from cohesion_separation / attraction_repulsive-term
                # boid_pos_x, boid_pos_y = boid.get_position()
                # neighbor_pos_x, neighbor_pos_y = flock[j].get_position()
                dx, dy = relative_position(boid, flock[j])

                cohesion_separation_x += (
                    cohesion_separation_gain_psi * dx
                )  # (neighbor_pos_x - boid_pos_x)
                cohesion_separation_y += (
                    cohesion_separation_gain_psi * dy
                )  # (neighbor_pos_y - boid_pos_y)

    ### Calculate alignment (Position based!)
    sum_dx = 0.0
    sum_dy = 0.0

    for j in range(len(flock)):
        if j != i:
            distance = distance_between_agents(boid, flock[j])

            if 0 < distance < sensor_range:
                # Get current positions
                # boid_pos_x, boid_pos_y = boid.get_position()
                # neighbor_pos_x, neighbor_pos_y = flock[j].get_position()
                dx, dy = relative_position(boid, flock[j])

                # Last relative position
                rel_x_now = dx  # neighbor_pos_x - boid_pos_x
                rel_y_now = dy  # neighbor_pos_y - boid_pos_y

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

    alignment_x = 1 / t * sum_dx
    alignment_y = 1 / t * sum_dy

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
    for j in range(len(flock)):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:  # "0 <" to avoid dividing by zero later
                neighbor_count += 1  # Count number of neighbors

    ### Calculate time-dependt alignment gain phi
    phi = 0.0
    if 0 < t <= 1 / k:
        phi = neighbor_count / t
    elif t > 1 / k:
        phi = k * neighbor_count

    #### Calculate cohesion separation
    cohesion_separation_x, cohesion_separation_y = 0.0, 0.0
    for j in range(len(flock)):
        if i != j:
            distance = distance_between_agents(boid, flock[j])
            if 0 < distance < sensor_range:
                # Calculate cohesion-seperation gain:
                cohesion_separation_gain_psi = 1 - (delta * neighbor_count) / distance

                # Then calculate cohesion_separation control input from cohesion_separation / attraction_repulsive-term
                # boid_pos_x, boid_pos_y = boid.get_position()
                # neighbor_pos_x, neighbor_pos_y = flock[j].get_position()
                dx, dy = relative_position(boid, flock[j])

                cohesion_separation_x += (
                    cohesion_separation_gain_psi + phi
                ) * dx  # (neighbor_pos_x - boid_pos_x)
                cohesion_separation_y += (
                    cohesion_separation_gain_psi + phi
                ) * dy  # (neighbor_pos_y - boid_pos_y)

    ### Calculate alignment (Position based!)
    rel_x_0 = 0.0
    rel_y_0 = 0.0

    for j in range(len(flock)):
        if j != i:
            distance = distance_between_agents(boid, flock[j])

            if 0 < distance < sensor_range:
                # Get initial positions
                boid_pos_x_init, boid_pos_y_init = boid.get_initial_position()
                neighbor_pos_x_init, neighbor_pos_y_init = flock[
                    j
                ].get_initial_position()
                # Initial relative position
                rel_x_0 += neighbor_pos_x_init - boid_pos_x_init
                rel_y_0 += neighbor_pos_y_init - boid_pos_y_init

    alignment_x = phi * rel_x_0
    alignment_y = phi * rel_y_0

    ### Control input for boid i
    control_input_x = cohesion_separation_x - alignment_x
    control_input_y = cohesion_separation_y - alignment_y

    return control_input_x, control_input_y


def update(flock, t, gamma_t, MAX_SPEED, mode):
    # def update(flock, t, gamma_t):

    # Update time step
    dt = 0.10
    t += dt

    # Update each boid individually
    for i in range(len(flock)):
        # Extract current boid
        boid_og = flock[i]

        # Range of sensor/boid/bird
        sensor_range = 60

        # Sepration gain
        delta = 7.5

        # Update speed for boid i
        vx, vy = boid_og.get_velocity()

        # Different modes:
        if mode == "velocity":  # Velocity based control
            u_x_vel_based, u_y_vel_based = control_input_velocity_based(
                boid_og, flock, sensor_range, delta
            )
            vx += dt * u_x_vel_based
            vy += dt * u_y_vel_based
        elif mode == "position":  # Position based control
            u_x_pos_based, u_y_pos_based = control_input_position_based_NO_threshold(
                boid_og, flock, sensor_range, delta, t
            )
            vx += dt * u_x_pos_based
            vy += dt * u_y_pos_based
        elif mode == "position_threshold":  # Position based control with threshold
            u_x_pos_based_threshold, u_y_pos_based_threshold = (
                control_input_position_based_with_threshold(
                    boid_og, flock, sensor_range, delta, t, k=0.1
                )
            )
            vx += dt * u_x_pos_based_threshold
            vy += dt * u_y_pos_based_threshold
        else:
            raise ValueError(f"Unknown mode: {mode}")

        # speed limiter
        speed = np.sqrt(vx**2 + vy**2)
        if speed > MAX_SPEED:
            vx = (vx / speed) * MAX_SPEED
            vy = (vy / speed) * MAX_SPEED

        # Execute update for boid i
        flock[i].update_velocity(vx, vy)
        flock[i].update_position(dt)  # points.set_data(x, y)

    # Metric
    gamma_sum = 0.0
    for i in range(len(flock)):
        boid_i = flock[i]
        gamma_sum += directional_alignment_metric(
            boid_i, flock, neighbor=range(len(flock))
        )

    gamma_value = gamma_sum / len(flock)
    gamma_t.append(gamma_value)

    # print("t: ", t)
    return round(t, 1)
    # print(gamma_value)


def main():
    args = parse_args()

    NUMBER_OF_AGENTS = args.agents
    STEPS = args.steps
    MAX_SPEED = args.max_speed
    HEIGHT = args.height
    WIDTH = args.width
    MODE = args.mode

    EPOCHS = args.epochs
    for epoch in range(EPOCHS):
        # White background
        img = (
            np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
        )  # for at få en hvid baggrund
        imgclear = (
            np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
        )  # for at få en hvid baggrund

        blueColor = (255, 50, 50)
        agentRadius = 3  # for drawing

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
        # Time passed
        T = 0.1

        # trace plot ting
        traces = [[] for _ in range(NUMBER_OF_AGENTS)]

        # Do the simulation
        while T < STEPS:
            # while True:
            img = imgclear.copy()  # to clear the image

            T = update(flock, T, gamma_t, MAX_SPEED, MODE)
            # print(T)
            # update(flock, T, gamma_t, MAX_SPEED)

            # trace save points (test)
            for i in range(NUMBER_OF_AGENTS):
                xBoid, yBoid = flock[i].get_position()
                cv2.circle(img, (int(xBoid), int(yBoid)), agentRadius, blueColor, -1)
                traces[i].append((xBoid, yBoid))

            cv2.imshow("Window", img)

            # print(traces)

            if cv2.waitKey(1) == ord("q"):
                break

        cv2.destroyAllWindows()

        gamma_smoothed = exponential_smoothing(gamma_t, alpha=0.1)
        plt.plot(gamma_smoothed)
        plt.savefig(f"gamma_t_plot_epoch_{epoch}.png")
        plt.close()

        # Plot trace
        plt.figure()
        for i in range(NUMBER_OF_AGENTS):
            if len(traces[i]) > 1:
                xs = [p[0] for p in traces[i]]
                ys = [p[1] for p in traces[i]]
                n = len(xs)
                # print("\n")
                for j in range(n - 1):
                    alpha = 0.20 + 0.75 * (j / (n - 4))
                    # print(alpha)
                    plt.plot(
                        xs[j : j + 2],
                        ys[j : j + 2],
                        color="blue",
                        alpha=alpha,
                        linewidth=1.2,
                    )
        plt.xlim(0, WIDTH)
        plt.ylim(0, HEIGHT)
        plt.gca().set_aspect("equal")
        # plt.savefig("trace_plot.png")
        plt.savefig(f"trace_plot_epoch_{epoch}.png")
        plt.close()


# Main
main()
