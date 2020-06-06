import cvxpy as cp
import numpy as np

state_size = 6
no_drones = 10


class Dispatch:
    def __init__(self, drones_list):
        self.drones_list = drones_list
        drone_track = {}
        for drone in self.drones_list:
            drone_track[drone.id] = drone
        self.drone_track = drone_track

    def assign_tasks(self, jobs):
        drone_states = np.empty((state_size, no_drones))
        drone_count = 0
        for key in self.drone_track.keys():
            drone = self.drone_track[key]
            initial_state = drone.getinitialState().T  # assumes it is a col vector so transposes to be a row since it makes for clearer matrix algebra
            drone_states[drone_count, ] = initial_state
        assignment_matrix = cp.Variable((no_drones, no_drones), symmetric=True)
        assignment_matrix2 = cp.Variable((no_drones, no_drones), boolean=True)
        cost_function = cp.sum(cp.norm(assignment_matrix * drone_states - jobs, axis=0))
        constraints = [assignment_matrix @ np.ones((4, 1)) == 1,
                       assignment_matrix == assignment_matrix2]
        MILP_objective = cp.Minimize(cost_function)
        opt_problem = cp.Problem(MILP_objective, constraints)
        result = opt_problem.solve(solver=cp.MOSEK, verbose=True)
        print(opt_problem.status)

        # TODO Need to choose what to output from MILP
        return


# jobs = np.array([[5, 23],
#                  [3, 44],
#                  [4, 5],
#                  [1, 44]])
#
# initial_state = np.array([[1, 1],
#                  [3, 2],
#                  [4, 2],
#                  [1, 2]])
# no_drones = 4
#
# assignment_matrix = cp.Variable((no_drones, no_drones), symmetric=True)
# assignment_matrix2 = cp.Variable((no_drones, no_drones), boolean=True)
# cost_function = cp.sum(cp.norm(assignment_matrix * initial_state - jobs, axis=0))
# constraints = [assignment_matrix @ np.ones((4, 1)) == 1,
#                assignment_matrix == assignment_matrix2]
# MILP_objective = cp.Minimize(cost_function)
# opt_problem = cp.Problem(MILP_objective, constraints)
# result = opt_problem.solve(solver=cp.MOSEK, verbose=True)

