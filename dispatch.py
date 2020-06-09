import cvxpy as cp
import numpy as np
state_size = 6
no_drones = 10
drone_weight = 1.5  # kg
alpha = 46.7
beta = 26.9


class dispatch:
    def __init__(self, drones_list):
        self.drones_list = drones_list
        drone_track = {}
        for drone in self.drones_list:
            drone_track[drone.id] = drone
        self.drone_track = drone_track

    def get_assignment(self, jobs):
        # TODO need to include checking if particular drone is free for assignment; Jobs should
        #  now include package weight
        drone_states = np.empty((state_size, no_drones))
        drone_count = 0
        for key in self.drone_track.keys():
            drone = self.drone_track[key]
            current_state = drone.return_state().T  # assumes it is a col vector so transposes to be a row since it makes for clearer matrix algebra
            drone_states[drone_count, ] = current_state
            drone_count += 1
        assignment_matrix = cp.Variable((no_drones, no_drones), symmetric=True)  # no_drones X no_drones
        assignment_matrix2 = cp.Variable((no_drones, no_drones), boolean=True)
        cost_function = cp.sum(cp.norm(assignment_matrix * drone_states[:, 0:3] - jobs[:, 0:3], axis=1))
        drone_weights = drone_states[:, 3]
        package_weights = jobs[:, 3]
        constraints = [assignment_matrix @ np.ones((no_drones, 1)) == 1,
                       assignment_matrix == assignment_matrix2,
                       assignment_matrix @ (alpha * (drone_weights + package_weights) + beta) * assignment_matrix *
                       1.5 * (drone_states[:, 0:3] - jobs[:, 0:3]) <= assignment_matrix @ drone_states[:, 4]
                       ]  # this includes battery constraint to ensure drone can service trip
        MILP_objective = cp.Minimize(cost_function)
        opt_problem = cp.Problem(MILP_objective, constraints)
        result = opt_problem.solve(solver=cp.MOSEK, verbose=True)
        print(opt_problem.status)
        return assignment_matrix.value, drone_states
        # NOW WE CAN ASSIGN JOBS MULTIPLE WAYS

    def assign_tasks(self, jobs):
        A_matrix, drone_states = self.get_assignment(jobs)
        drone_assign = A_matrix @ drone_states
        for i in range(no_drones):
            drone_with_task = drone_assign[i, ]
            drone_id = drone_with_task[-1]
            drone = self.drone_track[drone_id]
            drone.task = jobs[i, ]  # this contains coordinates of job destination for that drone

    def updated_drones_dict(self):
        # TODO will the drones stored in dictionary need to be updated?
        pass

    def available(self):
        available_drones = []
        for drone in self.drones_list:
            if drone.status == 0 or drone.status == 4:
                available_drones.append(drone)
        return available_drones


def update_tasks(time, incoming_task_list, task_list):
    if incoming_task_list[0][0] == time:
        temp_list = incoming_task_list.pop(0)
        temp_list = temp_list[1]
        for i in range(0, len(temp_list)):
            task_list.append(temp_list[i])
            print("Assigned task")
    return
