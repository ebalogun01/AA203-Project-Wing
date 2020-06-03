
class Dispatch:
    def __init__(self, drones_list):
        self.drones_list = drones_list
        drone_track = {}
        for drone in self.drones_list:
            drone_track[drone.id] = drone
        self.drone_track = drone_track


