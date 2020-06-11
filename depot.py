# Simple depot class implementing depot location, capacity and task_list


class Depot(object):
    def __init__(self, id, location, max_capacity = 100, 
                 curr_capacity = 0, task_list = []):
        self.id = id
        # location is a 2D coordinate
        self.location = location
        self.max_capacity = max_capacity
        self.curr_capacity = curr_capacity
        # self.task_list = task_list
        
    def add_drone(self):
        if self.curr_capacity == self.max_capacity:
            print("Error adding drone at depot ID: ", id)
        self.curr_capacity += 1
        return 
    
    def remove_drone(self):
        if self.curr_capacity == 0:
            print("Error removing drone at depot ID: ", id)
        self.curr_capacity -= 1
        
    '''
    def add_task(self, task):
        # here task represents target location
        self.task_list.append(task)
        
    def remove_task(self, task):
        self.task_list.remove(task)
    '''