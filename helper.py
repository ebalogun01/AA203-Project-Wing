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
                if new_loc[0]+2 > obs[0][0] and new_loc[0]-2 < obs[1][0] and \
                new_loc[1]+2 > obs[0][1] and new_loc[1]-2 < obs[1][1]:
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
        print(new_job)
        np.append(pending_jobs, new_job)
    return pending_jobs

def assign_paths(drones_list, paths_lookup, grid_lo, grid_hi, obs_grid):
    grid_size = grid_hi[0] - grid_lo[0]  # Assumes cube grid
    for drone in drones_list:
        if drone.status == 2:
            init = tuple(drone.position[0:2])
            target = tuple(drone.destination[0:2])
            init_idx = init[0] * grid_size + init[1]
            target_idx = target[0] * grid_size + target[1]
            if paths_lookup[init_idx][target_idx] is not None:
                drone.target_path = paths_lookup[init_idx][target_idx]
            else:
                A_star_ = AStar(grid_lo, grid_hi, init, target, obs_grid)
                A_star_.solve()
                drone.target_path = A_star_.path
                if drone.target_path == None:
                    print("Infeasible A-star path for drone ID:", drone.id)
                paths_lookup[init_idx][target_idx] = A_star_.path
    return

# Previous code for testing simulated assignment
#for i in range(0, 2):
#    drone = drones_list[i]
#    drone.destination = delivery_locs[i]
#    drone.status = 2
#    depot_idx = depot_locs.tolist().index(drone.position[0:2].tolist())
#    delivery_idx = delivery_locs.tolist().index(drone.destination.tolist())
#    print(depot_idx, delivery_idx)
#    drone.target_path = paths_lookup.a_star_paths_[0].path_list[depot_idx][delivery_idx]