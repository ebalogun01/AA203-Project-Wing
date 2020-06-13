# World sim helper functions

import numpy as np
from astar import AStar

def pick_location(grid_lo, grid_hi, obstacle_footprints, depot_locs):
# Returns random [x,y,0] location for delivery that is not a depot or obstacle
    inside = True
    buffer = 2
    while inside:
        x = np.random.randint(grid_lo[0]+buffer, grid_hi[0]-buffer)
        y = np.random.randint(grid_lo[1]+buffer, grid_hi[1]-buffer)
        new_loc = np.array([x, y])
        print(new_loc)

        if new_loc in depot_locs[:,0:2]:
            # Same location as depot, choose again
            print("Depot here")
            continue

        inside = False
        for obs in obstacle_footprints:
            if not inside:
                if new_loc[0]+4 > obs[0][0] and new_loc[0]-4 < obs[1][0] and \
                new_loc[1]+4 > obs[0][1] and new_loc[1]-4 < obs[1][1]:
                    # Inside an obstacle
                    print("Inside obstacle")
                    inside = True
                    break

    return np.append(new_loc, 0)

def update_tasks(pending_jobs, grid_lo, grid_hi, obstacle_footprints, depot_locs):
    prob_new_job = 0.2
    a = np.random.randint(10)  # Uniform random int
    if a < 10*prob_new_job:  # Implements probability function
        new_loc = pick_location(grid_lo, grid_hi, obstacle_footprints, depot_locs)
        if len(pending_jobs) == 0:
            job_id = 1
        else:
            job_id = pending_jobs[-1][4] + 1
        package_weight = np.random.randint(5)
        new_job = np.append(np.append(new_loc, package_weight), job_id)
        print("Adding new job: ", new_job)
        pending_jobs = np.append(pending_jobs, new_job.reshape([1,5]), axis = 0)
    return pending_jobs

def assign_paths(drones_list, depot_list, paths_lookup, grid_lo, grid_hi, obs_grid):
    grid_size = grid_hi[0] - grid_lo[0]  # Assumes cube grid
    for drone in drones_list:
        if drone.status == 2:
            # Check if the drone is going to a depot
            if drone.pickup_depot is not None and \
            not np.array_equal(drone.pickup_depot.location, drone.position):  # drone is not at its assigned depot
                print("Drone ID: ", drone.id, " not at pickup depot", drone.pickup_depot.location, drone.position)
                drone.destination = drone.pickup_depot.location
                drone.pickup_depot = None  # set to None, since it is being processed
            elif drone.task is not None:
                # Not going to depot, implies task has to be assigned
                drone.destination = drone.task[0:3]  # x,y,z location
                drone.weight += drone.task[3]  # assign package weight
                print("Drone ID: ", drone.id, " assigned task: ", drone.destination, " weight: ", drone.weight)
                drone.task = None  # set to None, since it is being processed

            init = tuple(drone.position[0:2])
            target = tuple(drone.destination[0:2])
            print("Assigning path, drone ID: ", drone.id, " Init: ", init,
                  " Target: ", target)
            init_idx = int(init[0] * grid_size + init[1])
            target_idx = int(target[0] * grid_size + target[1])
            if paths_lookup[init_idx][target_idx] is not None:
                print("Path already exists in lookup")
                drone.target_path = paths_lookup[init_idx][target_idx]
            else:
                A_star_ = AStar(grid_lo, grid_hi, init, target, obs_grid)
                A_star_.solve()
                drone.target_path = A_star_.path
                if drone.target_path is None:
                    print("Infeasible A-star path for drone ID:", drone.id)
                    return -1
                else:
                    # append 2D target path with height = 50 next
                    drone.target_path = np.asarray(drone.target_path, dtype = int)
                    drone.target_path = np.append(drone.target_path, 50*np.ones([len(drone.target_path),1]), 1)
                    # Add final drop off location at height 0 below final point
                    actual_dropoff = np.append(drone.target_path[-1][0:2],0)
                    actual_dropoff = np.reshape(actual_dropoff, (-1,3))
                    drone.target_path = np.append(drone.target_path, actual_dropoff, axis = 0)
                    paths_lookup[init_idx][target_idx] = drone.target_path
            drone.status = 3  # in-transit, will be picked up by dynamics
            # tracking index init for dynamics
            drone.tracking_index = 1

    return 0

def check_collisions_offset_path(intransit_drones_list):
# check if two drones are getting close and offset paths in Z to avoid collision
# if two drones are very close within 1 unit, return an error
    for i in range(0, len(intransit_drones_list)-1):
        for j in range(i+1, len(intransit_drones_list)):
            drone1 = intransit_drones_list[i]
            drone2 = intransit_drones_list[j]
            dist = np.linalg.norm(drone1.position - drone2.position)
            if dist < 0.4:
                print("Error! Drones are too close")
                return -1
            elif dist < 2:  # Buffer for path correction in Z
                # Check which drone is roughly in the lower section of its traj
                print("Drone ID: ", drone1.id, " and ID: ", drone2.id, " are getting close")
                drone_to_change = None
                if drone1.tracking_index / len(drone1.target_path) < 0.8:
                    drone_to_change = drone1
                elif drone2.tracking_index / len(drone2.target_path) < 0.8:
                    drone_to_change = drone2
                update_buffer = 5
                curr_index = drone_to_change.tracking_index
                offset_array = np.zeros([update_buffer, 3])
                offset_array[:,2] += 0.1  # offset by units

                ### Comment this out if you see issues
                # drone_to_change.target_path[curr_index:curr_index+update_buffer] += offset_array
                return 1
    return 0

# Previous code for testing simulated assignment
#for i in range(0, 2):
#    drone = drones_list[i]
#    drone.destination = delivery_locs[i]
#    drone.status = 2
#    depot_idx = depot_locs.tolist().index(drone.position[0:2].tolist())
#    delivery_idx = delivery_locs.tolist().index(drone.destination.tolist())
#    print(depot_idx, delivery_idx)
#    drone.target_path = paths_lookup.a_star_paths_[0].path_list[depot_idx][delivery_idx]
