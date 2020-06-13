# This will be used to generate grid, assign drone paths, set expected offline
# costs (which will change online)
import numpy as np
from astar import DetOccupancyGrid2D
from astar import AStar
from drone import Drone, rollout_dynamics, max_charge
from obstacle import obstacle
from depot import Depot
from dispatch import Dispatch
from helper import update_tasks, assign_paths, check_collisions_offset_path
from visualization import plot_path
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as manimation
matplotlib.use("Agg")

enable_plot = True

# Grid Parameters
grid_size = 100
grid_lower_left = (0, 0)
grid_upper_right = (grid_size, grid_size)
grid_height = grid_size

# Obstacle footprints as corners of rectangles - bottom left, top right
# The last element of each obstacle entry represents the height
obstacle_footprints = [((10, 5), (50, 20), 50),
                       ((65, 15), (75, 30), 50),
                       ((25, 30), (30, 45), 60),
                       ((35, 60), (60, 80), 60),
                       ((75, 50), (90, 60), 70)]
obstacles = obstacle(obstacle_footprints)

# Assume single depot at (0,0) to start with
depot_locs = np.array([[0, 0, 0], [95, 95, 0], [10, 95, 0]])
depot_list = [None] * len(depot_locs)
for i in range(0, len(depot_locs)):
    depot_list[i] = Depot(i, depot_locs[i])

# List of all delivery locations
delivery_locs = np.array([[99, 50, 0], [3, 99, 0], [98, 98, 0], [97, 97, 0]])

num_drones = 1

# Define grid sections according to heights of operation in 2D
heights_of_oper = [0, 50]  # [50, 60, 70, 80, 90]
obs_grid_list = [None] * len(heights_of_oper)
for i in range(0, len(heights_of_oper)):
    obs_grid_list[i] = DetOccupancyGrid2D(
        grid_upper_right[0] - grid_lower_left[0],
        grid_upper_right[1] - grid_lower_left[1],
        obstacles.section(heights_of_oper[i]))

# Initialize drones list at depot with empty delivery locations
drones_list = []
for i in range(num_drones):
    init_position = depot_locs[0]  # Append the initial Z height of the drone
    init_velocity = np.array([0, 0, 0])
    new_drone = Drone(i, init_position, init_velocity, [], 1, max_charge)
    drones_list.append(new_drone)

# Initialize dispatch
dispatch = Dispatch(drones_list, depot_list)

# jobs entries are of the format [x_loc, y_loc, z_loc, package_weight, job_id]
jobs = np.array([[1, 1, 0, 0.6, "001"],
                 [3, 3, 0, 0.8, "002"],
                 [10, 10, 0, 0.7, "003"],
                 [15, 18, 0, 0.3, "004"]])
# Temporary - Updating just to get run.py to compile
jobs = np.array([[99, 50, 0, 0.5, 1],
                 [3, 99, 0, 0.7, 2]])

# Inititialize Animation
FFMPegWriter = manimation.writers['ffmpeg']
writer = FFMPegWriter(fps=10)
fig = plt.figure(1)

# Initialize pending_jobs
pending_jobs = jobs

paths_lookup = [[None] * grid_size * grid_size] * (grid_size * grid_size)

max_time = 500
with writer.saving(fig,"animation.mp4",max_time):
    for time in range(0, max_time):
        # Run the world simulation here and break on error
        # Errors can be collision with obstacle OR out of charge, etc.
        print("Time: ", time)

        # Randomly add random jobs to the pending job list with probability 0.2
        pending_jobs = update_tasks(pending_jobs, grid_lower_left, grid_upper_right,
                                    obstacle_footprints, depot_locs)

        print("Number of available drones: ", len(dispatch.available()),
              "charging drones: ", len(dispatch.charging_drones()),
              "in-transit drones: ", len(dispatch.intransit_drones()))

        # Run Task Assignment Function
        # if time == 0:
        print("Pending jobs length is: ", len(pending_jobs))
        pending_jobs = dispatch.assign_tasks(pending_jobs)

        # Assign paths to drones with status = 2
        result = assign_paths(drones_list, depot_list, paths_lookup,
                              grid_lower_left, grid_upper_right, obs_grid_list[1])

        if result == -1:
            exit()

        drones_list = rollout_dynamics(drones_list)

        #esult = check_collisions_offset_path(dispatch.intransit_drones())
        #if result == -1:
        #    break

        if enable_plot:
            plot_path(drones_list,depot_list,obs_grid_list[0])
            #plt.draw()
            #plt.pause(.001)
            writer.grab_frame()
            plt.clf()

        time += 1
        if time == max_time:
            sum_cost = 0
            sum_tasks = 0
            for drone in drones_list:
                print("Drone ID: ", drone.id, " cost: ", drone.energy_used, 
                      "tasks completed: ", drone.completed_tasks)
                sum_cost += drone.energy_used
                sum_tasks += drone.completed_tasks
            print("Mean cost: ", sum_cost/num_drones)
            print("Total tasks: ", sum_tasks)
            print("Simulation Complete!")
            break