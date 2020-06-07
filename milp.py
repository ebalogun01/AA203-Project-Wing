import cvxpy as cp

"""
X -> x_ij=1 if drone i is assigned to delivery j
cost -> total length of delivery routes
d -> d_ij is the distance for drone i to deliver to request j

"""
def run_milp(drones_list, delivery_test, depot_locations, paths):
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



X = cp.Variable(m,n,boolean=True)
cost = cp.sum(cp.multiply(X,d))
constraints = 
p = cp.Problem(cp.Minimize(cost))
p.solve()



"""
Need to define:
    available_drones    -> list of drones not currently delivering a package
    path_dist           -> function that returns a* path length b/w 2 points
"""
