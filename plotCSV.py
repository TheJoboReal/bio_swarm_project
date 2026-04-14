import numpy as np
import math
import argparse
import matplotlib.pyplot as plt
import cv2
from agent import *

import csv



# Hvad skal den kunne
# læse 1 eller flere csv filer ind 
# plotte i forskellige farver. 
#   velocity based          --> blå
#   position based with thr --> Rød/orange
#   position based no thr   --> gul
#   
# 


# def parse_args():
#     parser = argparse.ArgumentParser(description="Boids Simulation")
#     parser.add_argument("--agents", type=int, default=DEFAULT_NUMBER_OF_AGENTS, help="Number of agents in the simulation")
#     parser.add_argument("--steps", type=int, default=DEFAULT_STEPS, help="Number of simulation steps")
#     parser.add_argument("--max_speed", type=float, default=DEFAULT_MAX_SPEED, help="Maximum speed of agents")
#     parser.add_argument("--epochs", type=int, default=DEFAULT_EPOCHS, help="Number of epochs")
#     parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="Window height")
#     parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Window width")
#     parser.add_argument("--mode", type=str, default="velocity", choices=["velocity", "position", "position_threshold"], help="Simulation mode: 'velocity', 'position', or 'position_threshold'")
#     return parser.parse_args()

def load_csv_to_list(filename):
    data = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            data.append([float(val) for val in row])
    return data



def plot_alone(fileName):
    # fileName = 'mode_velocity_seedAtEnd_4_steps_100_agents_50_gamma.csv'
    gamma_t = load_csv_to_list(fileName)
    
    steps = len(gamma_t[0])

    for run in gamma_t:
        plt.plot(run)
        

    dt = 0.1 # vores sample tid i simuleringen

    plt.xlabel(f'Total period {steps * dt} s')
    plt.ylabel("Gamma")
    plt.savefig(fileName + "_plot.png")



def plot_together(fileName1, fileName2, fileName3):
    gamma_t_1 = load_csv_to_list(fileName1)
    # start by take avg of each file
    # print(gamma_t_1)
    arr = np.array(gamma_t_1)
    arr_mean = np.mean(arr, axis=0)
    # print(arr)
    # print(arr.shape)
    # print(arr_mean)
    # print(arr_mean.shape)
    plt.plot(arr_mean, label="position (no thr)")

    gamma_t_2 = load_csv_to_list(fileName2)
    arr = np.array(gamma_t_2)
    arr_mean = np.mean(arr, axis=0)
    plt.plot(arr_mean, label="position (thr)")
    
    gamma_t_3 = load_csv_to_list(fileName3)
    arr = np.array(gamma_t_3)
    arr_mean = np.mean(arr, axis=0)
    plt.plot(arr_mean, label="velocity")
    
    steps = len(gamma_t_1[0])
    dt = 0.1 # vores sample tid i simuleringen
    
    plt.legend()
    plt.xlabel(f'Total period {steps * dt} s')
    plt.ylabel("Gamma")
    plt.savefig("All_3_modes" + "_plot.png")



def main():


    fileName_1 = 'mode_position_seedAtEnd_5_steps_30_agents_50_gamma.csv' # position
    fileName_2 = 'mode_position_threshold_seedAtEnd_5_steps_30_agents_50_gamma.csv' # position with threshold
    fileName_3 = 'mode_velocity_seedAtEnd_5_steps_30_agents_50_gamma.csv' # velocity mode
    plot_together(fileName_1, fileName_2, fileName_3)

    # plot_alone(fileName)



# Main
main()
