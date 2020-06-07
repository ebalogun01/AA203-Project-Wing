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
    [(delivery_idx,id),...] ->  returns a list of tuples of the index of the
                                delivery in the delivery_list & the id of the
                                drone assigned to it

"""
def run_milp(available_drones, delivery_list, depot_locs, paths):
    ## fill in matrix of distances, d
    m = len(available_drones)
    n = len(delivery_list)
    d = np.zeros((m,n))
    for i,drone in available_drones:
        for j,delivery in delivery_list:
            if drone.status==0:
                d[i,j] = path_dist(drone.position,delivery)
            else if drone.status==4:
                for depot in depot_locs:
                    d = path_dist(drone.position,depot)+path_dist(depot,delivery)
                    if d<d[i,j]:
                        d[i,j]==d

    ## CVX solve
    X = cp.Variable(m,n,boolean=True)
    cost = cp.sum(cp.multiply(X,d))
    constraints = [cp.sum(X,axis=0)<=1,cp.sum(X,axis=1)<=1]
    p = cp.Problem(cp.Minimize(cost),constraints)
    p.solve()

    ## return list of assignment pairs
    X_val = X.value
    assignments = []
    for i in range(m):
        j = np.argwhere(X_val[i,:]>0)
        if j.shape[0]>0:
            assignment.append((j[0,0],available_drones[i].id))
    return assignments
