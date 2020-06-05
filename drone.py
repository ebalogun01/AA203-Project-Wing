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
        self.path = None
        self.status = 0    # 0: free, 1: wait_path 2: to_dest, 3: to_charg, 4: to_depot
        self.noise_directions = self.generate_directions()

    def add_package(self, package_weight):
        self.weight += package_weight

    def add_destination(self, new_dest):
        self.destination = new_dest
        
    def generate_directions(self):
        direction_list = []
        for i in range(-1,2):
            for j in range(-1,2):
                for k in range(-1,2):
                    if(i == 0 and j == 0 and k == 0):
                        continue
                    else:
                        direction_list.append((i,j,k))
        return direction_list

    #def constraints(self):
    #    self.charge[0] == self.initial_Q
    #    self.charge[1:n] = self.charge[0:n-1] - self.weight * gamma - lamda * u
    #    self.charge >= 0
        
    def step(self, u):
        # Assume u is a 3x1 force vector
        # Apply wind disturbance to the control input force
        # Assume no disturbance (p=0.5), random disturbance (p = 0.5)
        random_direction = np.random.randint(len(self.noise_directions))
        u = u + 0.1 * np.random.randint(2) * self.noise_directions[random_direction]
        self.velocity = self.velocity + dt * u
        self.position = self.position + dt * self.velocity
        self.position = np.rint(self.position)  # Round pos to nearest ints