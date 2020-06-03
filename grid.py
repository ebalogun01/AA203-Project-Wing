# This will be used to generate grid, assign drone paths, set expected offline 
# costs (which will change online)
from astar import DetOccupancyGrid2D
from astar import AStar
from drone import drone

# Obstacle footprints as corners of rectangles - bottom left, top right
obstacle_footprints = [
    ((3,3),(5,5)),
    ((1,10),(2,11))]

# TODO: Eventually merge these two in an obstacle class, easier to handle
obstacle_heights = [5,15]

# TODO change the city grid to a list based on certain discrete planned
# heights of operation of the drones, ~4-5 values for heights
grid_lo = (0,0)
grid_hi = (20,20)
obs_grid = DetOccupancyGrid2D(grid_hi[0]-grid_lo[0],\
                               grid_hi[1]-grid_lo[1],obstacle_footprints)

# Assume single depot at (0,0) to start with
depot_locs = [(0,0)]

# Initialize drones list at depot with empty delivery locations
num_drones = 10
drones_list = []
for i in range(num_drones):
    init_position = depot_locs[0]
    temp = list(init_position)
    temp.append(0)  # Append the initial Z height of the drone
    init_position = tuple(temp)
    new_drone = drone(i, init_position, [], 1, 100, 100)
    drones_list.append(new_drone)
    
# TODO This list of delivery locations needs to come from MILP solution
# TODO Need to implement a priority queue data structure somewhere
delivery_list = [(19,10),(3,19)]

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
        A_star_ = AStar(grid_lo, grid_hi, drone.position[0:2], \
                        drone.destination, obs_grid)
        A_star_.solve()
        drone.path = A_star_.path
        print("Drone ID: ", drone.id, " path ", drone.path)
        
# TODO implement trajectory following drone dynamics with some wind noise
