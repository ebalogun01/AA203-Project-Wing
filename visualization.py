import matplotlib.pyplot as plt
import numpy as np


def plot_depots(depots,fig_num=1):
    plt.figure(fig_num)
    for depot_ in depots:
        plt.scatter(depot_.location[0],depot_.location[1],marker="D",s=75,color="orange")
        plt.annotate("depot", np.array(depot_.location[:2]) + np.array([-5, -5]), fontsize=8)
    #plt.axis([0, width, 0, height])
    #plt.axis('off')

def plot_path(drones, depots, obstacles,fig_num=1):
    """Plots the path found in path and the obstacles"""

    obstacles.plot(fig_num)
    plot_depots(depots)

    for drone in drones:
        if drone.status==3:
            solution_path = np.array(drone.target_path)
            if len(solution_path):
                plt.plot(solution_path[:,0],solution_path[:,1], '--',color="green", linewidth=1, zorder=10)
            plt.scatter(drone.destination[0], drone.destination[1], color="green", s=30, zorder=10)
            plt.scatter(drone.position[0],drone.position[1], color="purple", s=50, zorder=10)
            plt.annotate("drone%d" %drone.id, np.array(drone.position[:2]) + np.array([.2, .2]), fontsize=8)
    #plt.annotate(r"$x_{goal}$", np.array(self.x_goal) + np.array([.2, .2]), fontsize=16)
    #plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

    plt.axis([-10, 10+obstacles.width, -10, 10+obstacles.height])
    #plt.axis('off')
    #plt.show()
