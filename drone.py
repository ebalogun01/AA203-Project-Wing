class drone(object):

    def __init__(self,center,destination,weight,charge,speed=13,radius=1):
        self.center = center            #position of drone center w.r.t. the map
        self.destination = destination  #package destination or depot
        self.weight = weight            #weight of drone + package
        self.charge = charge            #current battery charge
        self.speed = speed              #constant speed of drone during delivery
        self.radius = radius            #distance from center to widest point
        self.path = None

    def add_package(self,package_weight):
        self.weight += package_weight

    def add_destination(self,new_dest):
        self.destination = new_dest
