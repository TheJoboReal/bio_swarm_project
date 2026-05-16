# bio_swarm_project
## Creators / Authors:
This project was created by Kasper Bloch Mathiasen (kmath22@student.sdu.dk),  Aksel Møller-Hansen (akmoe22@student.sdu.dk) and Harald Bay Baastrup (hniel22@student.sdu.dk).
The project was done as part of the course "Bio-Inspired Autonomous Systems", where the task was to replicate the paper "Position-Based Flocking for Persistent Alignment without Velocity Sensing" by Jond et al., which implemented a alignment methods without velocity sensing. 


## Requirements:
The following libraries needs to be installed:
```
opencv-python
numpy
matplotlib
```

## Instructions on how to run:
The following table show the possibilities when using argparse.

| Argument | Default | Description |
|---|---:|---|
| `--agents` | `30` | Number of agents in the simulation |
| `--steps` | `50` | Number of simulation steps |
| `--max_speed` | `2` | Maximum speed allowed for each agent |
| `--runs` | `1` | Number of repeated simulation runs |
| `--seed` | `0` | Random seed. Use a non-zero value for repeatable runs |
| `--cv2` | `1` | Enable or disable OpenCV visualization. Use `0` to disable |
| `--height` | `700` | Simulation window height |
| `--width` | `700` | Simulation window width |
| `--mode` | `velocity` | Control mode: `velocity`, `position`, or `position_threshold` |


An example experiment could be:
```
python main.py --agents 50 --steps 100 --max_speed 2 --seed 20 --cv2 0 --runs 200 --mode velocity
python main.py --agents 50 --steps 100 --max_speed 2 --seed 20 --cv2 0 --runs 200 --mode position
python main.py --agents 50 --steps 100 --max_speed 2 --seed 20 --cv2 0 --runs 200 --mode position_threshold
```

## Output from simulations:
Each simulation writes three CSV files. The file names include the mode, number of runs, number of steps, and number of agents.
```
mode_<MODE>_runs_<RUNS>_steps_<STEPS>_agents_<AGENTS>_gamma.csv
mode_<MODE>_runs_<RUNS>_steps_<STEPS>_agents_<AGENTS>_interagent_distance.csv
mode_<MODE>_runs_<RUNS>_steps_<STEPS>_agents_<AGENTS>_average_agent_speeds.csv
```

## Plotting results
After generating CSV files for all three modes, run:
```
python plotCSV.py
```
Please note, that the plotCSV.py expects runs = 200, steps = 100, and agents = 50. 