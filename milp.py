import cvxpy as cp

"""
run_milp:   minimize total distance travelled by all drones
            output determines delivery-drone assignment
input:
    available_drones -> list of drone objects that are free @ a depot or
                        on their way back from a previous delivery
    delivery_list    -> list of coordinates of delivery requests
    depot_locs       -> list of coordinates of depot locations
    paths            -> table with precomputed path lengths
variables:
    X -> x_ij=1 if drone i is assigned to delivery j
    cost -> total length of delivery routes
    d -> d_ij is the distance for drone i to deliver to request j

output:
    [(delivery_idx,ID),...] ->  returns a list of tuples of the index of the
                                delivery in the delivery_list & the ID of the
                                drone assigned to it

"""


# def run_milp(available_drones, task_list, delivery_list, depot_list, paths):
#     """fill in matrix of distances, d let the drones that are at depot be m1, drones in transit to depot be m2
#     let number of depots be p the number of rows of distance matrix should be (m1 + m2 * p)"""
#     m1 = 0
#     m2 = 0
#     for drone in available_drones:
#         if drone.status == 0:
#             m1 += 1
#         else:
#             m2 += 1
#     m = m1 + m2 * len(depot_list)
#     n = len(task_list)
#     d = np.zeros((m, n))
#     for i, drone in available_drones:
#         for j, task in task_list:
#             delivery_idx = delivery_list.index(task)
#             if drone.status == 0:  # drone is at depot waiting for task
#                 # Assume lowest height for now
#                 depot_idx = depot_list.index(drone.position)
#                 d[i, j] = paths.a_star_paths_[0].path_list[depot_idx][delivery_idx]
#             elif drone.status == 4:  # drone is on way back to depot
#                 for depot in depot_list:
#                     # Need to use Euclidean distance to depot here
#                     # since the drone is in-transit, cannot lookup distance from paths
#                     dist_to_depot = np.linalg.norm(drone.position[0:2] - depot.location)
#                     dist_to_task = paths.a_star_paths_[0].path_list[depot.ID][delivery_idx]
#                     d = dist_to_depot + dist_to_task
#                     if d[i, j] == 0 or d < d[i, j]:
#                         d[i, j] = d
#
#     ## CVX solve
#     X = cp.Variable(m, n, boolean=True)
#     cost = cp.sum(cp.multiply(X, d))
#     constraints = [cp.sum(X, axis=0) <= 1, cp.sum(X, axis=1) <= 1]
#     p = cp.Problem(cp.Minimize(cost), constraints)
#     p.solve()
#
#     ## return list of assignment pairs
#     X_val = X.value
#     assignments = []
#     for i in range(m):
#         j = np.argwhere(X_val[i, :] > 0)
#         if j.shape[0] > 0:
#             assignments.append((j[0, 0], available_drones[i].ID))
#     return assignments
