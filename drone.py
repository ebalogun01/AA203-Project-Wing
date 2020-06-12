# TODO we need to convert A-star to inputs, A-star will give us.
# Keep track of the time it takes to get to the destination
# Note: We will be using the Hexacopter in the drone delivery papers since all properties and parameters are readily
# available. Also, we will use linear approximation for the power consumed as a function of weight as this is
# a reasonable approx. as long as our package weights don't go beyond the drone weight. Parameters are obtained per
# Kevin Et. Al

import numpy as np

#
dt = 1
rho = 1.204  # kg/m^3
rotor_blade_area = 0.2  # m^2 propeller blades effective area
drone_weight = 1.5  # kg
alpha = 46.7
beta = 26.9
g = 9.81  # gravity


class Drone(object):
    def __init__(self, ID, position, velocity, destination, weight, charge):
        self.id = ID  # ID number to distinguish b/w drones
        self.position = position  # position (x,y,z) of drone center w.r.t. the map
        self.velocity = velocity  # velocity (vx,vy,vz) of the drone
        self.destination = destination  # package destination or depot
        self.weight = weight  # weight of drone + package
        self.charge = charge  # current battery charge
        self.target_path = None
        self.tracking_index = None  # needed for traj following
        self.status = 0  # 0: free, 1: wait_path 2: to_dest, 3: to_charg, 4: to_depot
        self.task = None
        self.pickup_depot = None

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
        return np.vstack([self.position, self.weight, self.charge, self.id])

    def step(self, u):
        # Assume u is a 3x1 force vector
        # Apply wind disturbance to the control input force
        # Assume no disturbance (p=0.5), random disturbance (p = 0.5)
        self.velocity = self.velocity + dt * u / self.weight \
                        + dt * np.array([0, 0, -g])
        self.position = self.position + dt * self.velocity
        if self.position[2] < 0:
            self.position[2] = 0
        self.charge -= (alpha * self.weight + beta) * dt  # this constant power is consumed per time-step
        print(" Drone charge left is {}".format(self.charge))
        # self.position = np.rint(self.position)  # Round pos to nearest ints


def rollout_dynamics(drones_list):
    # TODO: Add dynamics here
    for drone in drones_list:
        if drone.status >= 2:
            curr_target = drone.target_path[drone.tracking_index]

            kp = np.array([0.5, 0.5, 0.5])
            kd = np.array([0.1, 0.1, 1])
            u = np.multiply(kp, (curr_target - drone.position)) \
                - np.multiply(kd, drone.velocity)
            u = u + np.array([0, 0, drone.weight * 9.81]) + 0.1 * np.random.randint(2)
            u = np.maximum(np.minimum(u, np.array([5, 5, 20])), np.array([-5, -5, 0]))

            drone.step(u)
            print(drone.id, drone.position, drone.destination)
            dist_to_curr_target = np.linalg.norm(drone.position - curr_target)
            dist_to_target = np.linalg.norm(drone.position - drone.target_path[len(drone.target_path) - 1])
            if dist_to_target < 0.1:
                print("Drone ID: ", drone.id, " reached target")
                drone.position = drone.target_path[len(drone.target_path) - 1]
                drone.status = 0
                break
            if drone.tracking_index == len(drone.target_path) - 1 or dist_to_curr_target > 3:
                continue
            else:
                drone.tracking_index += 1
