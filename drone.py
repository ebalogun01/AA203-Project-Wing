#TODO we need to convert A-star to inputs, A-star will give us. Keep track of the time it takes to get to the destination
import numpy as np

dt = 1

class drone(object):
    def __init__(self,id,position,velocity,destination,weight,charge):
        self.id = id                    #id number to distinguish b/w drones
        self.position = position        #position (x,y,z) of drone center w.r.t. the map
        self.velocity = velocity        #velocity (vx,vy,vz) of the drone
        self.destination = destination  #package destination or depot
        self.weight = weight            #weight of drone + package
        self.charge = charge            #current battery charge
        self.target_path = None
        self.tracking_index = None      # needed for traj following
        self.status = 0    # 0: free, 1: wait_path 2: to_dest, 3: to_charg, 4: to_depot
        
        # define variables to store the history for plotting
        self.u_history = None
        self.position_history = None
        self.velocity_history = None

    def add_package(self, package_weight):
        self.weight += package_weight

    def drop_package(self, package_weight):
        self.weight -= package_weight

    def add_destination(self, new_dest):
        self.destination = new_dest

    def return_state(self):
        return np.vstack([self.position, self.velocity, self.charge, self.weight, self.id])

    #def constraints(self):
    #    self.charge[0] == self.initial_Q
    #    self.charge[1:n] = self.charge[0:n-1] - self.weight * gamma - lamda * u
    #    self.charge >= 0

    def step(self, u):
        # Assume u is a 3x1 force vector
        # Apply wind disturbance to the control input force
        # Assume no disturbance (p=0.5), random disturbance (p = 0.5)
        u = u + 0.1 * np.random.randint(2)
        self.velocity = np.maximum(np.minimum(self.velocity + dt * u, np.array([2,2,2])), np.array([-2,-2,-2]))
        self.position = self.position + dt * self.velocity
        #self.position = np.rint(self.position)  # Round pos to nearest ints

def rollout_dynamics(drones_list):
    # TODO: Add dynamics here
    for drone in drones_list:
        if drone.status >= 2:
            curr_target = drone.path[drone.tracking_index]
            u = 0.5 * (curr_target - drone.position) - 0.1 * (drone.velocity)
            drone.step(u)
            dist_to_curr_target = np.linalg.norm(drone.position - curr_target)
            dist_to_target = np.linalg.norm(drone.position - drone.path[len(drone.path)-1])
            if dist_to_target < 0.1:
                print("Drone ID: ", drone.id, " reached target")
                drone.position = drone.path[len(drone.path)-1]
                break
            if drone.tracking_index == len(drone.path)-1 or dist_to_curr_target > 3:
                continue
            else:
                drone.tracking_index += 1
