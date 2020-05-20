import numpy as np

EPSILON_OMEGA = 1e-3

def angle_diff(a, b):
    a = a % (2. * np.pi)
    b = b % (2. * np.pi)
    diff = a - b
    if np.size(diff) == 1:
        if np.abs(a - b) > np.pi:
            sign = 2. * (diff < 0.) - 1.
            diff += sign * 2. * np.pi
    else:
        idx = np.abs(diff) > np.pi
        sign = 2. * (diff[idx] < 0.) - 1.
        diff[idx] += sign * 2. * np.pi
    return diff

def compute_dynamics(x, u, dt, compute_jacobians=True):
    """
    Compute Turtlebot dynamics (unicycle model).

    Inputs:
                        x: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
        compute_jacobians: bool         - compute Jacobians Gx, Gu if true.
    Outputs:
         g: np.array[3,]  - New state after applying u for dt seconds.
        Gx: np.array[3,3] - Jacobian of g with respect to x.
        Gu: np.array[3,2] - Jacobian of g with respect ot u.
    """
    ########## Code starts here ##########
    # TODO: Compute g, Gx, Gu
    g = np.zeros(3)
    g[2] = u[1]*dt+x[2]
    g[0] = dt*u[0]*np.cos(x[2]+u[1]*dt/2)+x[0]
    g[1] = dt*u[0]*np.sin(x[2]+u[1]*dt/2)+x[1]
    Gx = np.array([[1,0,-dt*u[0]*np.sin(u[1]*dt/2+x[2])],[0,1,dt*u[0]*np.cos(u[1]*dt/2+x[2])],[0,0,1]])
    Gu = np.array([[dt*np.cos(x[2]+u[1]*dt/2),-dt**2*u[0]/2*np.sin(x[2]+u[1]*dt/2)],[dt*np.sin(x[2]+u[1]*dt/2),dt**2*u[0]/2*np.cos(x[2]+u[1]*dt/2)],[0,dt]])
    ########## Code ends here ##########

    if not compute_jacobians:
        return g

    return g, Gx, Gu

def transform_line_to_scanner_frame(line, x, tf_base_to_camera, compute_jacobian=True):
    """
    Given a single map line in the world frame, outputs the line parameters
    in the scanner frame so it can be associated with the lines extracted
    from the scanner measurements.

    Input:
                     line: np.array[2,] - map line (alpha, r) in world frame.
                        x: np.array[3,] - pose of base (x, y, theta) in world frame.
        tf_base_to_camera: np.array[3,] - pose of camera (x, y, theta) in base frame.
         compute_jacobian: bool         - compute Jacobian Hx if true.
    Outputs:
         h: np.array[2,]  - line parameters in the scanner (camera) frame.
        Hx: np.array[2,3] - Jacobian of h with respect to x.
    """
    alpha, r = line

    ########## Code starts here ##########
    # TODO: Compute h, Hx
    x_c = x[0]+tf_base_to_camera[0]*np.cos(x[2])
    y_c = x[1]+tf_base_to_camera[1]*np.sin(x[2])
    theta_c = tf_base_to_camera[2]+x[2]
    #r_c = -x_c*np.cos(alpha)-y_c*np.sin(alpha)+r
    r_c = r-np.sqrt(x[0]**2+x[1]**2)*np.cos(np.arctan2(x[1],x[0])-alpha)-np.sqrt(tf_base_to_camera[0]**2+tf_base_to_camera[1]**2)*np.cos(np.arctan2(tf_base_to_camera[1],tf_base_to_camera[0])-alpha+x[2])
    alpha_c = angle_diff(alpha,theta_c)
    alpha_c += np.pi if np.sign(r_c)==-1 else 0
    h = np.array([alpha_c,np.abs(r_c)])
    np.sqrt(tf_base_to_camera[0]**2+tf_base_to_camera[1]**2)*np.cos(np.arctan2(tf_base_to_camera[1],tf_base_to_camera[0])-alpha+x[2])
    Hx = np.array([[0,0,-1],[-np.sign(r_c)*np.cos(alpha),-np.sign(r_c)*np.sin(alpha), np.sign(r_c)*np.sqrt(tf_base_to_camera[0]**2+tf_base_to_camera[1]**2)*np.sin(np.arctan2(tf_base_to_camera[1],tf_base_to_camera[0])-alpha+x[2])]])
    ########## Code ends here ##########

    if not compute_jacobian:
        return h

    return h, Hx


def normalize_line_parameters(h, Hx=None):
    """
    Ensures that r is positive and alpha is in the range [-pi, pi].

    Inputs:
         h: np.array[2,]  - line parameters (alpha, r).
        Hx: np.array[2,n] - Jacobian of line parameters with respect to x.
    Outputs:
         h: np.array[2,]  - normalized parameters.
        Hx: np.array[2,n] - Jacobian of normalized line parameters. Edited in place.
    """
    alpha, r = h
    if r < 0:
        alpha += np.pi
        r *= -1
        if Hx is not None:
            Hx[1,:] *= -1
    alpha = (alpha + np.pi) % (2*np.pi) - np.pi
    h = np.array([alpha, r])

    if Hx is not None:
        return h, Hx
    return h
