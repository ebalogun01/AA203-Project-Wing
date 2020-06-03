class drone(object):

    def __init__(self,id,position,destination,weight,charge):
        self.id = id                    #id number to distinguish b/w drones
        self.position = position        #position of drone center w.r.t. the map
        self.destination = destination  #package destination or depot
        self.weight = weight            #weight of drone + package
        self.charge = charge            #current battery charge
        self.path = None

    def add_package(self,package_weight):
        self.weight += package_weight

    def add_destination(self,new_dest):
        self.destination = new_dest
