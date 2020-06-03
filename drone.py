#TODO we need to convert A-star to inputs, A-star will give us. Keep track of the time it takes to get to the destination

class drone(object):
    def __init__(self,id,position,destination,weight,charge, initial_Q):
        self.id = id                    #id number to distinguish b/w drones
        self.position = position        #position of drone center w.r.t. the map
        self.destination = destination  #package destination or depot
        self.weight = weight            #weight of drone + package
        self.charge = charge            #current battery charge
        self.path = None
        self.initial_Q = initial_Q
        self.status = 0    # 0: free, 1: wait_path 2: to_dest, 3: to_charg, 4: to_depot

    def add_package(self, package_weight):
        self.weight += package_weight

    def add_destination(self, new_dest):
        self.destination = new_dest

    def constraints(self):
        self.charge[0] == self.initial_Q
        self.charge[1:n] = self.charge[0:n-1] - self.weight * gamma - lamda * u
        self.charge >= 0