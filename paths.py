# This implements the lookup table for shortest paths at each height
import numpy as np

class paths(object):
    def __init__(self, grid_lower, grid_upper, heights, obs_grid_list, 
                 depot_locs, delivery_locs):
        self.euclidean_paths_ = euclidean_paths(depot_locs, delivery_locs)
        
        self.a_star_paths_ = []
        for i in range(0, len(obs_grid_list)):
            self.a_star_paths_[i] = a_star_paths(grid_lower, grid_upper, heights[i],
                                         obs_grid_list[i],
                                         depot_locs, delivery_locs)
            
class euclidean_paths(object):
    def __init__(self, depot_locs, delivery_locs):
        self.distances = np.array([depot_locs, delivery_locs])
        for i in range(0, len(depot_locs)):
            for j in range(0, len(delivery_locs)):
                init = np.asarray(depot_locs[i])
                target = np.asarray(delivery_locs[j]);
                self.distances[i][j] = np.linalg.norm(target - init)

class a_star_paths(object):
    def __init__(self, grid_lo, grid_hi, obs_grid, depot_locs, delivery_locs):
        self.distances = np.array([depot_locs, delivery_locs])
        self.path_list = [[None] * len(delivery_locs)] * depot_locs
        self.height = height
        
        for i in range(0, len(depot_locs)):
            for j in range(0, len(delivery_locs)):
                init = depot_locs[i]
                target = delivery_locs[j];
                A_star_ = AStar(grid_lo, grid_hi, init, target, obs_grid)
                A_star_.solve()
                self.path_list[i][j] = A_star_.path
                # TODO: Make sure diagonals are treated longer in A-star
                self.distances[i][j] = A_star_.actual_distance()