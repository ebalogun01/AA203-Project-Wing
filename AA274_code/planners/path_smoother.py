import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    t = np.zeros(len(path))
    for i in range(1,len(path)):
        t[i] = t[i-1]+ np.sqrt((path[i][0]-path[i-1][0])**2+(path[i][1]-path[i-1][1])**2)/V_des
        #path_length += np.sqrt((path[i][0]-path[i-1][0])**2+(path[i][1]-path[i-1][1])**2)
    tf = t[-1]
    x = map(list,zip(*path))[0]
    y = map(list,zip(*path))[1]
    spl_x = scipy.interpolate.splrep(t,x,s=alpha)
    spl_y = scipy.interpolate.splrep(t,y,s=alpha)
    t_smoothed = np.arange(0,tf,dt)
    traj_smoothed = np.zeros((len(t_smoothed),7))
    traj_smoothed[:,0] = scipy.interpolate.splev(t_smoothed,spl_x)
    traj_smoothed[:,1] = scipy.interpolate.splev(t_smoothed,spl_y)
    traj_smoothed[:,3] = scipy.interpolate.splev(t_smoothed,spl_x,der=1)
    traj_smoothed[:,4] = scipy.interpolate.splev(t_smoothed,spl_y,der=1)
    traj_smoothed[:,2] = np.arctan2(traj_smoothed[:,4],traj_smoothed[:,3])
    traj_smoothed[:,5] = scipy.interpolate.splev(t_smoothed,spl_x,der=2)
    traj_smoothed[:,6] = scipy.interpolate.splev(t_smoothed,spl_y,der=2)
    ########## Code ends here ##########

    return traj_smoothed, t_smoothed
