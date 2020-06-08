# This will be used to generate grid, assign drone paths, set expected offline 
# costs (which will change online)
import numpy as np
from astar import DetOccupancyGrid2D
from astar import AStar
from drone import drone
from obstacle import obstacle
from paths import paths
from depot import depot

# Grid Parameters
grid_lower_left = (0,0)
grid_upper_right = (100,100)
grid_height = 100

# Obstacle footprints as corners of rectangles - bottom left, top right
# The last element of each obstacle entry represents the height
obstacle_footprints = [((3,0), (99,20), 50),
                       ((3,3), (5,5),   60),
                       ((1,10),(2,11),  70)]
obstacles = obstacle(obstacle_footprints)

# Assume single depot at (0,0) to start with
depot_locs = np.array([[0,0], [90,10]])
depot_list = [None] * len(depot_locs)
for i in range(0, len(depot_locs)):
    depot_list[i] = depot(i, depot_locs[i])

# List of all delivery locations
delivery_locs = np.array([[99,50],[3,99],[98,98],[97,97]])

num_drones = 10

# Define grid sections according to heights of operation in 2D
heights_of_oper = [0] # [50, 60, 70, 80, 90]
obs_grid_list = [None] * len(heights_of_oper)
for i in range(0, len(heights_of_oper)):
    print(obstacles.section(heights_of_oper[i]))
    obs_grid_list[i] = DetOccupancyGrid2D(\
                                grid_upper_right[0]-grid_lower_left[0],\
                                grid_upper_right[1]-grid_lower_left[1],\
                                obstacles.section(heights_of_oper[i]))

# Initialize drones list at depot with empty delivery locations
drones_list = []
for i in range(num_drones):
    init_position = np.append(depot_locs[0], 0)  # Append the initial Z height of the drone
    init_velocity = np.array([0,0,0])
    new_drone = drone(i, init_position, init_velocity, [], 1, 100)
    drones_list.append(new_drone)
    
# Pre-calculate A-star and Euclidean shortest paths for each height of operation
# Note that this is the lookup table for both MILP and trajectory following
paths_lookup = paths(grid_lower_left, grid_upper_right,
                    heights_of_oper, obs_grid_list, depot_locs, delivery_locs)
'''
while(True):
    # Run the world simulation here and break on error
    # Errors can be collision with obstacle OR out of charge, etc.
    update_tasks()  # This will add tasks to the depot class
    
    # run_milp
    run_milp(drones_list, delivery_list, depot_list, paths_lookup)
    
    rollout_dynamics()
'''
'''    
# TODO This list of delivery locations needs to come from MILP solution
# TODO Need to implement a priority queue data structure somewhere

for i in range(len(delivery_list)):
    for j in range(len(drones_list)):
        drone = drones_list[j]
        if(drone.status == 0): # drone does not have an active task and is free
            drone.destination = delivery_list[i]
            drone.status = 1
            break

# Compute A-star trajectory for each drone that is waiting for path update
for i in range(len(drones_list)):
    drone = drones_list[i]
    if(drone.status == 1):
        A_star_ = AStar(grid_lo, grid_hi, tuple(drone.position[0:2].tolist()), \
                        tuple(drone.destination.tolist()), obs_grid)
        A_star_.solve()
        drone.path = A_star_.path
        drone.path = np.asarray(drone.path, dtype = int)
        drone.path = np.append(drone.path, np.zeros([len(drone.path),1]), 1)
        # print("Drone ID: ", drone.id, " path ", drone.path)
        
# TODO implement trajectory following drone dynamics with some wind noise
drone = drones_list[0]
actual_path = np.empty([0,3])
actual_velocity = np.empty([0,3])
index = 1
while(True):
    actual_path = np.append(actual_path, drone.position.reshape([1,3]), axis = 0)
    actual_velocity = np.append(actual_velocity, drone.velocity.reshape([1,3]), axis = 0)
    curr_target = drone.path[index]
    u = 0.5 * (curr_target - drone.position) - 0.1 * (drone.velocity)
    drone.step(u)
    dist_to_curr_target = np.linalg.norm(drone.position - curr_target)
    print(curr_target, drone.position, dist_to_curr_target)
    dist_to_target = np.linalg.norm(drone.position - drone.path[len(drone.path)-1])
    if dist_to_target < 0.1:
        print("Reached target")
        drone.position = drone.path[len(drone.path)-1]
        break
    if index == len(drone.path)-1 or dist_to_curr_target > 3:
        continue
    else:
        index += 1    

# TODO: remove later, for debugging only   
np.savetxt("target_path.txt", drone.path)
np.savetxt("actual_path.txt", actual_path) 
np.savetxt("actual_vel.txt", actual_velocity)
'''