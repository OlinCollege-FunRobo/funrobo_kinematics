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
        """
        Parameters (copied from RRTPlannerOptions)
        ----------
            max_step_size : float
                Maximum joint configuration step size for collision checking along path segments.
            max_connection_dist : float
                Maximum angular distance, in radians, for connecting nodes.
            rrt_connect : bool
                If true, enables the RRTConnect algorithm, which incrementally extends the most
                recently sampled node in the tree until an invalid state is reached.
            bidirectional_rrt : bool
                If true, uses bidirectional RRTs from both start and goal nodes.
                Otherwise, only grows a tree from the start node.
            rrt_star : bool
                If true, enables the RRT* algorithm to shortcut node connections during planning.
                This in turn will use the `max_rewire_dist` parameter.
            max_rewire_dist : float
                Maximum angular distance, in radians, to consider rewiring nodes for RRT*.
                If set to `np.inf`, all nodes in the trees will be considered for rewiring.
            max_planning_time : float
                Maximum planning time, in seconds.
            rng_seed : int, optional
                Sets the seed for random number generation. Use to generate deterministic results.
            fast_return : bool
                If True, return as soon as a solution is found. Otherwise continuing building the tree
                until max_planning_time is reached.
            goal_biasing_probability : float
                Probability of sampling the goal configuration itself, which can help planning converge.
            collision_distance_padding : float
                The padding, in meters, to use for distance to nearest collision.
        """
            
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