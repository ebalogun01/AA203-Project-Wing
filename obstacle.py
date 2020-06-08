# Class that contains constructor and helper functions for obstacles

class obstacle(object):
    def __init__(self, geometry):
        self.footprints = self.assign_footprints(geometry)
        self.heights = self.assign_heights(geometry)
        
    def assign_footprints(self, geometry):
        footprint = [None] * len(geometry)
        for i in range(0, len(geometry)):
            footprint[i] = [geometry[i][0],geometry[i][1]]
        return footprint
    
    def assign_heights(self, geometry):
        height = [None] * len(geometry)
        for i in range(0, len(geometry)):
            height[i] = geometry[i][2]
        return height
    
    def section(self, height):
        section_footprints = []
        for i in range(0, len(self.heights)):
            if (self.heights[i] >= height):
                section_footprints.append(self.footprints[i])
        return section_footprints
        