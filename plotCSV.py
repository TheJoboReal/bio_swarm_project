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
def get_last_word(filename):
    parts = filename.split("_")
    last_part = parts[-1]              # 'gamma.csv'
    name = last_part.replace(".csv", "")  # 'gamma'
    return name

def get_title_info(filename):
    parts = filename.replace(".csv", "").split("_")

    info = {}

    for i, part in enumerate(parts):
        if part == "steps":
            info["steps"] = parts[i+1]
        elif part == "agents":
            info["agents"] = parts[i+1]
        elif part == "runs":
            info["runs"] = parts[i+1]

    return info

def make_title(filename):
    info = get_title_info(filename)

    return f"{info['runs']} runs | {info['steps']} steps | {info['agents']} agents"



def load_csv_to_list(filename):
    data = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            data.append([float(val) for val in row])
    return data



def plot_alone(fileName):
    # fileName = 'mode_velocity_seedAtEnd_4_steps_100_agents_50_gamma.csv'
    gamma_t = get_last_word(fileName)
    name = load_csv_to_list(fileName)
    steps = len(gamma_t[0])

    for run in gamma_t:
        plt.plot(run)

    dt = 0.1 # vores sample tid i simuleringen

    plt.xlabel(f'Total period {steps * dt} s')
    plt.ylabel("Gamma")
    plt.savefig(fileName + "_plot.png")


# gamma 
def plot_together_gamma(fileName1, fileName2, fileName3):
    plt.figure()
    
    plotName = get_last_word(fileName1)

    gamma_t_1 = load_csv_to_list(fileName1)
    # start by take avg of each file
    # print(gamma_t_1)
    
    arr = np.array(gamma_t_1)
    arr_mean = np.mean(arr, axis=0)

    dt = 0.1 # vores sample tid i simuleringen
    time = np.arange(len(arr_mean)) * dt
    # print(arr)
    # print(arr.shape)
    # print(arr_mean)
    # print(arr_mean.shape)
    #plt.plot(arr_mean, label="position (no thr)")

    plt.plot(time, arr_mean, label="position (no thr)", color="blue")

    gamma_t_2 = load_csv_to_list(fileName2)
    arr = np.array(gamma_t_2)
    arr_mean = np.mean(arr, axis=0)
    # plt.plot(arr_mean, label="position (thr)")
    plt.plot(time, arr_mean, label="position (thr)", color="orange") 
    
    gamma_t_3 = load_csv_to_list(fileName3)
    arr = np.array(gamma_t_3)
    arr_mean = np.mean(arr, axis=0)
    # plt.plot(arr_mean, label="velocity")
    plt.plot(time, arr_mean, label="velocity", color="green")
    
    steps = len(gamma_t_1[0])
    
    plt.axhline(1, linestyle='--')
    
    plt.legend()
    plt.title(make_title(fileName1))
    plt.xlabel(f'Time [s]')
    plt.ylabel(r'Alignment score $\gamma$')
    plt.ylim(-1, 1.1) # to make it plot from -1 to 1
    plt.savefig(f"All_3_modes_{plotName}.png")
    plt.close()


# distance
def plot_together_distance(fileName1, fileName2, fileName3):
    plt.figure()
    
    plotName = get_last_word(fileName1)

    gamma_t_1 = load_csv_to_list(fileName1)
    # start by take avg of each file
    # print(gamma_t_1)
    arr = np.array(gamma_t_1)
    arr_mean = np.mean(arr, axis=0)

    dt = 0.1 # vores sample tid i simuleringen
    time = np.arange(len(arr_mean)) * dt

    plt.plot(time, arr_mean, label="position (no thr)", color="blue")

    gamma_t_2 = load_csv_to_list(fileName2)
    arr = np.array(gamma_t_2)
    arr_mean = np.mean(arr, axis=0)
    # plt.plot(arr_mean, label="position (thr)")
    plt.plot(time, arr_mean, label="position (thr)", color="orange") 
    
    gamma_t_3 = load_csv_to_list(fileName3)
    arr = np.array(gamma_t_3)
    arr_mean = np.mean(arr, axis=0)
    # plt.plot(arr_mean, label="velocity")
    plt.plot(time, arr_mean, label="velocity", color="green")
    
    steps = len(gamma_t_1[0])
    dt = 0.1 # vores sample tid i simuleringen
    
    plt.legend()
    plt.title(make_title(fileName1))
    plt.xlabel(f'Time [s]')
    plt.ylabel(r'Distance [m]')
    plt.savefig(f"All_3_modes_{plotName}.png")
    plt.close()


# speed
def plot_together_speed(fileName1, fileName2, fileName3):
    plt.figure()
    
    plotName = get_last_word(fileName1)

    gamma_t_1 = load_csv_to_list(fileName1)
    # start by take avg of each file
    # print(gamma_t_1)
    arr = np.array(gamma_t_1)
    arr_mean = np.mean(arr, axis=0)
    dt = 0.1 # vores sample tid i simuleringen
    time = np.arange(len(arr_mean)) * dt

    plt.plot(time, arr_mean, label="position (no thr)", color="blue")

    gamma_t_2 = load_csv_to_list(fileName2)
    arr = np.array(gamma_t_2)
    arr_mean = np.mean(arr, axis=0)
    # plt.plot(arr_mean, label="position (thr)")
    plt.plot(time, arr_mean, label="position (thr)", color="orange") 
    
    gamma_t_3 = load_csv_to_list(fileName3)
    arr = np.array(gamma_t_3)
    arr_mean = np.mean(arr, axis=0)
    # plt.plot(arr_mean, label="velocity")
    plt.plot(time, arr_mean, label="velocity", color="green")
    
    steps = len(gamma_t_1[0])
    dt = 0.1 # vores sample tid i simuleringen
    
    plt.legend()
    plt.title(make_title(fileName1))
    plt.xlabel(f'Time [s]')
    plt.ylabel(r'Average speed $[m/s]$')
    plt.savefig(f"All_3_modes_{plotName}.png")
    plt.close()




def main():
    # plot_alone(fileName)
    # Gamma
    fileName_1 = 'mode_position_runs_100_steps_100_agents_50_gamma.csv' # position
    fileName_2 = 'mode_position_threshold_runs_100_steps_100_agents_50_gamma.csv' # position with threshold
    fileName_3 = 'mode_velocity_runs_100_steps_100_agents_50_gamma.csv' # velocity mode
    plot_together_gamma(fileName_1, fileName_2, fileName_3)

    # Speed
    fileName_1 = 'mode_position_runs_100_steps_100_agents_50_average_agent_speeds.csv' # position
    fileName_2 = 'mode_position_threshold_runs_100_steps_100_agents_50_average_agent_speeds.csv' # position with threshold
    fileName_3 = 'mode_velocity_runs_100_steps_100_agents_50_average_agent_speeds.csv' # velocity mode
    plot_together_speed(fileName_1, fileName_2, fileName_3)

    # Distance
    fileName_1 = 'mode_position_runs_100_steps_100_agents_50_interagent_distance.csv' # position
    fileName_2 = 'mode_position_threshold_runs_100_steps_100_agents_50_interagent_distance.csv' # position with threshold
    fileName_3 = 'mode_velocity_runs_100_steps_100_agents_50_interagent_distance.csv' # velocity mode
    plot_together_distance(fileName_1, fileName_2, fileName_3)

# Main
main()
