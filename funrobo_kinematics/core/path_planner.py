from pinocchio.visualize import MeshcatVisualizer

import matplotlib.pyplot as plt

import pyroboplan.models.hiwonder as hiwonder
import pyroboplan.models.two_dof as two_dof
from pyroboplan.planning.rrt import RRTPlanner, RRTPlannerOptions

from scipy import interpolate
import numpy as np


class RobotPathPlanner():
    """
    A class for planning paths for a robot using RRT algorithm.
    """
    
    def __init__(self, robot_name="2-dof", obstacle_list=None):

        if robot_name == "2-dof":
            self.model, self.collision_model, self.visual_model = two_dof.load_models()
            two_dof.add_object_collisions(self.model, self.collision_model, self.visual_model, obstacle_list)
            self.ndof = 2
        elif robot_name == "hiwonder":
            self.model, self.collision_model, self.visual_model = hiwonder.load_models()
            hiwonder.add_object_collisions(self.model, self.collision_model, self.visual_model, obstacle_list)
            self.ndof = 5
        self.data = self.model.createData()
        self.collision_data = self.collision_model.createData()


    def plan_path(self, q_start, q_end, visualize=False):
        options = RRTPlannerOptions(
            max_step_size=0.05,
            max_connection_dist=0.2,
            rrt_connect=False,
            bidirectional_rrt=False,
            rrt_star=True,
            max_rewire_dist=1.0,
            max_planning_time=10.0,
            rng_seed=None,
            fast_return=True,
            goal_biasing_probability=0.1,
            collision_distance_padding=0.0,
        )
        planner = RRTPlanner(self.model, self.collision_model, options=options)
        
        # Search for a path
        self.path = planner.plan(q_start, q_end)

        if not self.path:
            print("[Planner Error]: No path found within the time limit.")
            return None

        if visualize:
            self.viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model, data=self.data)
            self.viz.initViewer(open=True)
            self.viz.loadViewerModel()
            self.viz.display(self.path[0])
            planner.visualize(self.viz, "ee", show_tree=True)

        return self.path


    def generate_trajectory(self, path, nsteps=50):
        """
        Discretizes a joint space path into a trajectory with position, velocity, and acceleration.
        """
        if not path:
            print("No path found to generate trajectory.")
            return None

        joint_values = [[th[i] for th in path] for i in range(self.ndof)]
        
        # ensure that then number of points is greater than the degree of the spline (default is 3)
        k = 3 if len(path) > 3 else len(path)-1

        tck,u = interpolate.splprep(joint_values, k=k, s=0)
        u = np.linspace(0, 1, num=nsteps, endpoint=True)

        self.t = np.linspace(0, 1, nsteps)
        
        out = interpolate.splev(u,tck)
        self.pos_traj = list(zip(*out))

        out = interpolate.splev(u, tck, der=1)
        self.vel_traj = list(zip(*out))

        # ensure that spline degree is greater than 2 before computing acceleration
        if k > 2:
            out = interpolate.splev(u, tck, der=2)
            self.acc_traj = list(zip(*out))
        else:
            self.acc_traj = [[0.0] * self.ndof for _ in range(nsteps)]

        return self.pos_traj
    
    
    def plot(self):
        """
        Plot the position, velocity, and acceleration trajectories.
        """

        self.labels = [f'axis{i+1}' for i in range(self.ndof)]

        self.fig = plt.figure()
        self.sub1 = self.fig.add_subplot(3,1,1)  # Position plot
        self.sub2 = self.fig.add_subplot(3,1,2)  # Velocity plot
        self.sub3 = self.fig.add_subplot(3,1,3)  # Acceleration plot

        self.fig.set_size_inches(8, 10)    
        self.fig.suptitle("Trajectory Generator", fontsize=16)

        colors = ['r', 'g', 'b', 'm', 'y']

        for i in range(self.ndof):
            # position plot
            theta = [th[i] for th in self.pos_traj]
            self.sub1.plot(self.t, theta, colors[i]+'o-', label=self.labels[i])
            self.sub1.set_ylabel('position', fontsize=15)
            self.sub1.grid(True)
            self.sub1.legend()
        
            # velocity plot
            theta = [th[i] for th in self.vel_traj]
            self.sub2.plot(self.t, theta, colors[i]+'o-', label=self.labels[i])
            self.sub2.set_ylabel('velocity', fontsize=15)
            self.sub2.grid(True)
            self.sub2.legend()

            # # acceleration plot
            theta = [th[i] for th in self.acc_traj]
            self.sub3.plot(self.t, theta, colors[i]+'o-', label=self.labels[i])
            self.sub3.set_ylabel('acceleration', fontsize=15)
            self.sub3.set_xlabel('Time (secs)', fontsize=18)
            self.sub3.grid(True)
            self.sub3.legend()

        plt.show()