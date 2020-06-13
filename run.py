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
depot_locs = np.array([[0, 0, 0], [50, 45, 0]])
depot_list = [None] * len(depot_locs)
for i in range(0, len(depot_locs)):
    depot_list[i] = Depot(i, depot_locs[i])

# List of all delivery locations
delivery_locs = np.array([[99, 50, 0], [3, 99, 0], [98, 98, 0], [97, 97, 0]])

num_drones = 10

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

max_time = 200
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

        result = check_collisions_offset_path(dispatch.intransit_drones())
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
            print("Simulation Complete!")
            break

'''
# TODO This list of delivery locations needs to come from MILP solution
# TODO Need to implement a priority queue data structure somewhere

for i in range(len(jobs)):
    for j in range(len(drones_list)):
        drone = drones_list[j]
        if(drone.status == 0): # drone does not have an active task and is free
            drone.destination = jobs[i][0:3]
            drone.status = 1
            break

# Compute A-star trajectory for each drone that is waiting for path update
for i in range(len(drones_list)):
    drone = drones_list[i]
    if(drone.status == 1):
        A_star_ = AStar(grid_lower_left, grid_upper_right, \
                        tuple(drone.position[0:2].tolist()), \
                        tuple(drone.destination[0:2].tolist()), obs_grid_list[0])
        A_star_.solve()
        drone.target_path = A_star_.path
        drone.target_path = np.asarray(drone.target_path, dtype = int)
        drone.target_path = np.append(drone.target_path, 50*np.ones([len(drone.target_path),1]), 1)
        actual_dropoff = np.append(drone.target_path[-1][0:2],0)
        actual_dropoff = np.reshape(actual_dropoff, (-1,3))
        print(actual_dropoff)
        drone.target_path = np.append(drone.target_path, actual_dropoff, axis = 0)
        print("Target path len: ", len(drone.target_path))
        # print("Drone ID: ", drone.ID, " path ", drone.path)

# Trajectory following implementation with wind noise
drone = drones_list[0]
actual_path = np.empty([0,3])
actual_velocity = np.empty([0,3])
actual_u = np.empty([0,3])
index = 1
# while(True):
for t in range(0,2000):
    actual_path = np.append(actual_path, drone.position.reshape([1,3]), axis = 0)
    actual_velocity = np.append(actual_velocity, drone.velocity.reshape([1,3]), axis = 0)
    curr_target = drone.target_path[index]
    kp = np.array([0.5, 0.5, 0.5])
    kd = np.array([0.1, 0.1, 1])
    u = np.multiply(kp, (curr_target - drone.position)) \
                                        - np.multiply(kd, (drone.velocity))
    u = u + np.array([0, 0, drone.weight*9.81]) + 0.1 * np.random.randint(2)
    u = np.maximum(np.minimum(u, np.array([5, 5, 20])), np.array([-5, -5, 0]))
    actual_u = np.append(actual_u, u.reshape([1,3]), axis = 0)
    drone.step(u)
    dist_to_curr_target = np.linalg.norm(drone.position - curr_target)
    print(curr_target, drone.position, dist_to_curr_target)
    dist_to_target = np.linalg.norm(drone.position - drone.target_path[-1])
    if dist_to_target < 0.1:
        print("Reached target")
        drone.position = drone.target_path[-1]
        break
    if index == len(drone.target_path)-1 or dist_to_curr_target > 3:
        continue
    else:
        index += 1

# TODO: remove later, for debugging only
np.savetxt("target_path.txt", drone.target_path)
np.savetxt("actual_path.txt", actual_path)
np.savetxt("actual_vel.txt", actual_velocity)
np.savetxt("actual_u.txt", actual_u)
'''
