# Class that contains constructor and helper functions for obstacles

class obstacle(object):
    def __init__(self, geometry):
        self.footprints = assign_footprints(geometry)
        self.heights = assign_heights(geometry)
        
    def assign_footprints(self, geometry):
        footprint = []
        for i in range(0, len(geometry)):
            footprint[i] = [(geometry[0][0],geometry[0][1])]
        return footprint
    
    def assign_heights(self, geometry):
        height = []
        for i in range(0, len(geometry)):
            height[i] = geometry[0][2]
        return height
    
    def section(self, height):
        section_footprints = []
        index = 0
        for obs_height in self.heights:
            if (obs_height >= height):
                section_footprints[index] = self.footprints[obs]
        return section_footprints
        