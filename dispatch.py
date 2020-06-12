import cvxpy as cp
import numpy as np

state_size = 6
depot_info_size = 4
no_drones = 10
drone_weight = 1.5  # kg
alpha = 46.7
beta = 26.9


class Dispatch:
    def __init__(self, drones_list, depots_list):
        self.drones_list = drones_list
        self.depots_list = depots_list
        drone_track = {}
        depot_track = {}
        for drone in self.drones_list:
            drone_track[drone.id] = drone
        self.drone_track = drone_track
        for depot in depots_list:
            depot_track[depot.id] = depot
        self.depot_track = depot_track

    def get_assignment(self, jobs):
        # TODO this allows all drones possibilities to go to any depot
        dim = 3
        jobs_count = jobs.shape[0]
        depot_states = np.empty((len(self.depots_list), depot_info_size))
        drone_states = np.empty((no_drones, state_size))

        drone_count = 0
        depot_count = 0
        available_drones = self.available()

        for drone in available_drones:
            current_state = drone.return_state()  # assumes it is a row
            drone_states[drone_count, ] = current_state
            drone_count += 1

        for depot in self.depots_list:
            depot_info = depot.get_depot_info()
            depot_states[depot_count, ] = depot_info
            depot_count += 1

        depot_loc_withID = np.repeat(depot_states, repeats=no_drones, axis=0)
        depot_loc = depot_loc_withID[:, 0:dim]

        assignment_matrix = cp.Variable((drone_count, drone_count), symmetric=True)  # no_drones X no_drones
        assignment_matrix2 = cp.Variable((drone_count, drone_count), boolean=True)
        depot_assigment = cp.Variable((drone_count, depot_loc.shape[0]), boolean=True)

        if drone_count > jobs_count:  # handles case where there are more drones than jobs
            drone_surplus = drone_count - jobs_count
            padding = np.repeat(np.array([0, 0, 0, 0, 0]), drone_surplus)
            padding = np.reshape(padding, (drone_surplus, dim + 2))
            jobs = np.vstack([jobs, padding])
        else:
            jobs = jobs[0:drone_count, :]
        no_jobs = jobs.shape[0]

        # cost_function = cp.sum(cp.norm(assignment_matrix * drone_states[:, 0:3] - jobs[:, 0:3], axis=1))
        cost_function = cp.sum(cp.norm(assignment_matrix * drone_states[:, 0:dim] - depot_assigment * depot_loc, axis=1) +
                               cp.norm(depot_assigment * depot_loc - jobs[:, 0:dim], axis=1))
        drone_weights = drone_states[:, dim:dim+1]
        package_weights = jobs[:, dim:dim+1]
        beta_vec = np.reshape(np.repeat(beta, drone_states.shape[0]), (drone_states.shape[0], 1))
        drone_battery = np.reshape(drone_states[:, dim + 1], (drone_count, 1))
        constraints = [assignment_matrix @ np.ones((no_drones, 1)) == 1,
                       assignment_matrix == assignment_matrix2,
                       # cp.multiply(alpha * (assignment_matrix @ drone_weights + package_weights) + beta_vec,
                       #             1.5 * cp.reshape(cp.norm(assignment_matrix @ drone_states[:, 0:dim] -
                       #                                      jobs[:, 0:dim], axis=1), (drone_count, 1)))
                       # <= assignment_matrix @ drone_battery,
                       # 1.5 * cp.norm((assignment_matrix @ drone_states[:, 0:dim] - jobs[:, 0:dim]), axis=1)) <=
                       # assignment_matrix @ drone_states[:, dim + 1],
                       cp.sum(depot_assigment, axis=0) <= 1, cp.sum(depot_assigment, axis=1) <= 1
                       ]  # this includes battery constraint to ensure drone can service trip
        MILP_objective = cp.Minimize(cost_function)
        opt_problem = cp.Problem(MILP_objective, constraints)
        result = opt_problem.solve(solver=cp.MOSEK, verbose=True)
        if opt_problem.status != 'optimal':
            print("Cannot dispatch all drones, removing min SOC drone")
            return
        print(opt_problem.status)
        return no_jobs, assignment_matrix.value, drone_states, depot_assigment.value @ depot_loc_withID
        # NOW WE CAN ASSIGN JOBS MULTIPLE WAYS

    def assign_tasks(self, jobs):  # Jobs should be an np array
        no_jobs, A_matrix, drone_states, assigned_depots = self.get_assignment(jobs)
        drone_assign = A_matrix @ drone_states
        for i in range(min(no_drones, no_jobs)):
            drone_with_task = drone_assign[i, ]
            depot_for_drone = assigned_depots[i, ]
            drone_id = drone_with_task[-1]
            depot_id = depot_for_drone[-1]
            drone = self.drone_track[drone_id]
            drone.status = 1
            drone.task = jobs[i, ]  # this contains coordinates of job destination and job ID for that drone
            drone.pickup_depot = self.depot_track[depot_id]
            self.drone_track[drone_id] = drone  # this is just to be double-sure that drone is updated.

    def get_assignments_astar(self, task_list, delivery_list, depot_list, paths):
        """fill in matrix of distances, d let the drones that are at depot be m1, drones in transit to depot be m2
        let number of depots be p the number of rows of distance matrix should be (m1 + m2 * p)"""
        available_drones = self.available()
        m1 = 0
        m2 = 0
        for drone in available_drones:
            if drone.status == 0:
                m1 += 1
            else:
                m2 += 1
        m = m1 + m2 * len(depot_list)
        n = len(task_list)
        d = np.zeros((m, n))
        for i, drone in available_drones:
            for j, task in task_list:
                delivery_idx = delivery_list.index(task)
                if drone.status == 0:  # drone is at depot waiting for task
                    # Assume lowest height for now
                    depot_idx = depot_list.index(drone.position)
                    d[i, j] = paths.a_star_paths_[0].path_list[depot_idx][delivery_idx]
                elif drone.status == 4:  # drone is on way back to depot
                    for depot in depot_list:
                        # Need to use Euclidean distance to depot here
                        # since the drone is in-transit, cannot lookup distance from paths
                        dist_to_depot = np.linalg.norm(drone.position[0:2] - depot.location)
                        dist_to_task = paths.a_star_paths_[0].path_list[depot.id][delivery_idx]
                        d = dist_to_depot + dist_to_task
                        if d[i, j] == 0 or d < d[i, j]:
                            d[i, j] = d

        ## CVX solve
        X = cp.Variable(m, n, boolean=True)
        cost = cp.sum(cp.multiply(X, d))
        constraints = [cp.sum(X, axis=0) <= 1, cp.sum(X, axis=1) <= 1]
        p = cp.Problem(cp.Minimize(cost), constraints)
        p.solve()

        ## return list of assignment pairs
        X_val = X.value
        assignments = []
        for i in range(m):
            j = np.argwhere(X_val[i, :] > 0)
            if j.shape[0] > 0:
                assignments.append((j[0, 0], available_drones[i].id))
        return assignments

    def updated_drones_dict(self):
        # TODO will the drones stored in dictionary need to be updated? probably not
        pass

    def remove_minSOC_drone(self):
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
