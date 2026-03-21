import numpy as np
import matplotlib.pyplot as plt

from scipy import interpolate


class MultiAxisTrajectoryGenerator():
    """
    Multi-axis trajectory generator for joint or task space trajectories.

    Supports linear, cubic, quintic polynomial, and trapezoidal velocity profiles.
    """
    
    def __init__(self, method="quintic",
                 mode="joint",
                 interval=[0,1],
                 ndof=1,
                 start_pos=None,
                 final_pos=None,
                 start_vel=None,
                 final_vel=None,
                 start_acc=None,
                 final_acc=None,
                 ):
        """
        Initialize the trajectory generator with the given configuration.

        Args:
            method (str): Type of trajectory ('linear', 'cubic', 'quintic', 'trapezoid').
            mode (str): 'joint' for joint space, 'task' for task space.
            interval (list): Time interval [start, end] in seconds.
            ndof (int): Number of degrees of freedom.
            start_pos (list): Initial positions.
            final_pos (list): Final positions.
            start_vel (list): Initial velocities (default 0).
            final_vel (list): Final velocities (default 0).
            start_acc (list): Initial accelerations (default 0).
            final_acc (list): Final accelerations (default 0).
        """

        self.T = interval[1]
        self.ndof = ndof
        self.t = None
        
        if mode == "joint":
            self.mode = "Joint Space"
            # self.labels = ['th1', 'th2', 'th3', 'th4', 'th5']
            self.labels = [f'axis{i+1}' for i in range(self.ndof)]
        elif mode == "task":
            self.mode = "Task Space"
            self.labels = ['x', 'y', 'z']
        
        # Assign positions and boundary conditions
        self.start_pos = start_pos
        self.final_pos = final_pos
        self.start_vel = start_vel if start_vel is not None else [0] * self.ndof
        self.final_vel = final_vel if final_vel is not None else [0] * self.ndof
        self.start_acc = start_acc if start_acc is not None else [0] * self.ndof
        self.final_acc = final_acc if final_acc is not None else [0] * self.ndof      

        # Select trajectory generation method
        if method == "linear":
            self.m = LinearInterp(self)
        elif method == "cubic":
            self.m = CubicPolynomial(self)
            self.m.solve(tf=self.T)
        elif method == "quintic":
            self.m = QuinticPolynomial(self)
            self.m.solve(tf=self.T)
        elif method == "trapezoid":
            self.m = TrapezoidVelocity(self)

    
    def generate(self, nsteps=100):
        """
        Generate the trajectory at discrete time steps.

        Args:
            nsteps (int): Number of time steps.
        Returns:
            list: List of position, velocity, acceleration for each DOF.
        """
        self.t = np.linspace(0, self.T, nsteps)
        return self.m.generate(tf=self.T, nsteps=nsteps)


    def plot(self):
        """
        Plot the position, velocity, and acceleration trajectories.
        """
        self.fig = plt.figure()
        self.sub1 = self.fig.add_subplot(3,1,1)  # Position plot
        self.sub2 = self.fig.add_subplot(3,1,2)  # Velocity plot
        self.sub3 = self.fig.add_subplot(3,1,3)  # Acceleration plot

        self.fig.set_size_inches(8, 10)    
        self.fig.suptitle(self.mode + " Trajectory Generator", fontsize=16)

        colors = ['r', 'g', 'b', 'm', 'y']

        for i in range(self.ndof):
            # position plot
            self.sub1.plot(self.t, self.m.X[i][0], colors[i]+'o-', label=self.labels[i])
            self.sub1.set_ylabel('position', fontsize=15)
            self.sub1.grid(True)
            self.sub1.legend()
        
            # velocity plot
            self.sub2.plot(self.t, self.m.X[i][1], colors[i]+'o-', label=self.labels[i])
            self.sub2.set_ylabel('velocity', fontsize=15)
            self.sub2.grid(True)
            self.sub2.legend()

            # acceleration plot
            self.sub3.plot(self.t, self.m.X[i][2], colors[i]+'o-', label=self.labels[i])
            self.sub3.set_ylabel('acceleration', fontsize=15)
            self.sub3.set_xlabel('Time (secs)', fontsize=18)
            self.sub3.grid(True)
            self.sub3.legend()

        plt.show()
        

# class MultiSegmentTrajectoryGenerator():
#     """
#     Multi-axis trajectory generator for joint or task space trajectories.

#     Supports linear, cubic, quintic polynomial, and trapezoidal velocity profiles.
#     """
    
#     def __init__(self, method="quintic",
#                  mode="joint",
#                  T=1,
#                  ndof=1,
#                  waypoints=None,
#                  nsteps=50
#                  ):
#         """
#         Initialize the trajectory generator with the given configuration.
#         """

#         self.T = T
#         self.ndof = ndof
#         self.t = None
#         self.waypoints = np.array(waypoints)
#         self.nwaypoints = len(waypoints)
#         self.nsteps = nsteps
        
#         if mode == "joint":
#             self.mode = "Joint Space"
#             # self.labels = ['th1', 'th2', 'th3', 'th4', 'th5']
#             self.labels = [f'axis{i+1}' for i in range(self.ndof)]
#         elif mode == "task":
#             self.mode = "Task Space"
#             self.labels = ['x', 'y', 'z']
        
#         # Assign positions and boundary conditions
#         self.start_pos = [0] * self.ndof
#         self.final_pos = [0] * self.ndof
#         self.start_vel = [0] * self.ndof
#         self.final_vel = [0] * self.ndof
#         self.start_acc = [0] * self.ndof
#         self.final_acc = [0] * self.ndof

#         # Select trajectory generation method
#         if method == "cubic":
#             self.m = CubicPolynomial(self)
#             self.solve()
#         elif method == "quintic":
#             self.m = QuinticPolynomial(self)
#             self.solve()
#         elif method == "bspline":
#             self.m = BSpline(self)
#             self.m.generate()

    
#     def solve(self):
#         self.traj_, self.trajectory = [], []

#         for i in range(self.nwaypoints-1):
#             self.m.start_pos = self.waypoints[i]
#             self.m.final_pos = self.waypoints[i+1]
#             self.m.start_vel = [0] * self.ndof if i == 0 else (self.waypoints[i] - self.waypoints[i-1]) / self.T
#             self.m.final_vel = (self.waypoints[i+1] - self.waypoints[i]) / self.T if i < self.nwaypoints-2 else [0] *  self.ndof
#             self.m.start_acc = [0] * self.ndof
#             self.m.final_acc = [0] * self.ndof
            
#             self.m.solve(t0=i*self.T, tf=(i+1)*self.T)
#             X = self.m.generate(t0=i*self.T, tf=(i+1)*self.T, nsteps=self.nsteps)
#             self.traj_.append(X)
        
#         q_, qd_, qdd_ = [], [], []
#         for j in range(self.ndof):
#             q, qd, qdd = [], [], []
#             for i in range(self.nwaypoints-1):
#                 q.extend(self.traj_[i][j][0])
#                 qd.extend(self.traj_[i][j][1])
#                 qdd.extend(self.traj_[i][j][2])
#             q_.append(q)
#             qd_.append(qd)
#             qdd_.append(qdd)
        
#         self.trajectory = [q_, qd_, qdd_]
#         # q_ = [all waypoints for DOF1, all waypoints for DOF2, ...]

    
#     def generate(self, nsteps=100):
#         """
#         Generate the trajectory at discrete time steps.

#         Args:
#             nsteps (int): Number of time steps.
#         Returns:
#             list: List of position, velocity, acceleration for each DOF.
#         """
#         self.t = np.linspace(0, self.T, nsteps)
#         return self.m.generate(nsteps=nsteps)
    
    
#     def get_joints_by_waypoints(self):
#         """
#         Get the joint positions at each waypoint.
#         """
#         jp = self.trajectory[0]  # position trajectory
#         traj = []
#         for j in range(len(jp[0])): # number of points in the trajectory
#             joint_values = []
#             for i in range(self.ndof):
#                 joint_values.append(jp[i][j])
#             traj.append(joint_values)

#         return traj

        
#     def plot(self):
#         """
#         Plot the position, velocity, and acceleration trajectories.
#         """
#         self.fig = plt.figure()
#         self.sub1 = self.fig.add_subplot(3,1,1)  # Position plot
#         self.sub2 = self.fig.add_subplot(3,1,2)  # Velocity plot
#         self.sub3 = self.fig.add_subplot(3,1,3)  # Acceleration plot

#         self.fig.set_size_inches(8, 10)    
#         self.fig.suptitle(self.mode + " Trajectory Generator", fontsize=16)

#         colors = ['r', 'g', 'b', 'm', 'y']

#         self.t = np.linspace(0, self.T*(len(self.waypoints)-1), self.nsteps*(len(self.waypoints)-1))

#         for i in range(self.ndof):
#             # position plot
#             self.sub1.plot(self.t, self.trajectory[0][i], colors[i]+'o-', label=self.labels[i])
#             self.sub1.set_ylabel('position', fontsize=15)
#             self.sub1.grid(True)
#             self.sub1.legend()
    
#             # velocity plot
#             self.sub2.plot(self.t, self.trajectory[1][i], colors[i]+'o-', label=self.labels[i])
#             self.sub2.set_ylabel('velocity', fontsize=15)
#             self.sub2.grid(True)
#             self.sub2.legend()

#             # acceleration plot
#             self.sub3.plot(self.t, self.trajectory[2][i], colors[i]+'o-', label=self.labels[i])
#             self.sub3.set_ylabel('acceleration', fontsize=15)
#             self.sub3.set_xlabel('Time (secs)', fontsize=18)
#             self.sub3.grid(True)
#             self.sub3.legend()

#         plt.show()  
    

class LinearInterp():
    """
    Linear interpolation between start and end positions.
    """

    def __init__(self, trajgen):
        self._copy_params(trajgen)
        self.solve()


    def _copy_params(self, trajgen):
        self.start_pos = trajgen.start_pos
        self.final_pos = trajgen.final_pos
        self.T = trajgen.T
        self.ndof = trajgen.ndof
        self.X = [None] * self.ndof

    
    def solve(self):
        pass  # Linear interpolation is directly computed in generate()
        

    def generate(self, nsteps=100):
        self.t = np.linspace(0, self.T, nsteps)
        for i in range(self.ndof): # iterate through all DOFs
            q, qd, qdd = [], [], []
            for t in self.t: # iterate through time, t
                q.append((1 - t/self.T)*self.start_pos[i] + (t/self.T)*self.final_pos[i])
                qd.append(self.final_pos[i] - self.start_pos[i])
                qdd.append(0)    
            self.X[i] = [q, qd, qdd]
        return self.X


class CubicPolynomial():
    """
    Cubic interpolation with position and velocity boundary constraints.
    """

    def __init__(self, trajgen):
        self._copy_params(trajgen)
        # self.solve()


    def _copy_params(self, trajgen):
        self.start_pos = trajgen.start_pos
        self.start_vel = trajgen.start_vel
        self.final_pos = trajgen.final_pos
        self.final_vel = trajgen.final_vel
        self.T = trajgen.T
        self.ndof = trajgen.ndof
        self.X = [None] * self.ndof

    
    def solve(self, t0=0, tf=0):
        self.A = np.array(
                [[1, t0, t0**2, t0**3],
                 [0, 1, 2*t0, 3*t0**2],
                 [1, tf, tf**2, tf**3],
                 [0, 1, 2*tf, 3*tf**2]
                ])
        self.b = np.zeros([4, self.ndof])

        for i in range(self.ndof):
            self.b[:, i] = [self.start_pos[i], self.start_vel[i],
                            self.final_pos[i], self.final_vel[i]]

        self.coeff = np.linalg.solve(self.A, self.b)
        

    def generate(self, t0=0, tf=0, nsteps=100):
        self.t = np.linspace(t0, tf, nsteps)

        for i in range(self.ndof): # iterate through all DOFs
            q, qd, qdd = [], [], []
            c = self.coeff[:,i]
            for t in self.t: # iterate through time, t
                q.append(c[0] + c[1] * t + c[2] * t**2 + c[3] * t**3)
                qd.append(c[1] + 2 * c[2] * t + 3 * c[3] * t**2)
                qdd.append(2 * c[2] + 6 * c[3] * t)    
            self.X[i] = [q, qd, qdd]
        return self.X.copy()


class QuinticPolynomial():
    """
    Quintic interpolation with position, velocity, and acceleration constraints.
    """

    def __init__(self, ndof=None):
        self.ndof = ndof

    
    def solve(self, q0, qf, qd0, qdf, qdd0, qddf, T):
        t0, tf = 0, T
        q0 = np.asarray(q0, dtype=float)
        qf = np.asarray(qf, dtype=float)
        qd0 = np.zeros_like(q0) if qd0 is None else np.asarray(qd0, dtype=float)
        qdf = np.zeros_like(q0) if qdf is None else np.asarray(qdf, dtype=float)
        qdd0 = np.zeros_like(q0) if qdd0 is None else np.asarray(qdd0, dtype=float)
        qddf = np.zeros_like(q0) if qddf is None else np.asarray(qddf, dtype=float)

        A = np.array(
                [[1, t0, t0**2, t0**3, t0**4, t0**5],
                [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                [1, tf, tf**2, tf**3, tf**4, tf**5],
                [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
                ])
            
        b = np.vstack([
            q0,
            qd0,
            qdd0,
            qf,
            qdf,
            qddf
        ])
        self.coeff = np.linalg.solve(A, b)
        

    def generate(self, t0=0, tf=0, nsteps=100):

        t = np.linspace(t0, tf, nsteps)
        X = np.zeros((self.ndof, 3, len(t)))
        for i in range(self.ndof):
            c = self.coeff[:, i]

            q = (c[0] + c[1] * t + c[2] * t**2 + c[3] * t**3 +  # position
                 c[4] * t**4 + c[5] * t**5)
            qd = (c[1] + 2 * c[2] * t + 3 * c[3] * t**2 +       # velocity
                  4 * c[4] * t**3 + 5 * c[5] * t**4)
            qdd = (2 * c[2] + 6 * c[3] * t + 12 * c[4] * t**2 +  # acceleration
                20 * c[5] * t**3)
 
            X[i, 0, :] = q
            X[i, 1, :] = qd
            X[i, 2, :] = qdd

        return t, X


class BSpline():
    """
    Basis spline (B-spline) interpolation between start and end positions.
    """

    def __init__(self, trajgen):
        self._copy_params(trajgen)


    def _copy_params(self, trajgen):
        self.nsteps = trajgen.nsteps
        self.ndof = trajgen.ndof
        self.waypoints = trajgen.waypoints
        self.T = trajgen.T
        self.X = [None] * self.ndof
    

    def generate(self):
        """
        Discretizes a joint space path into a trajectory with position, velocity, and acceleration.
        """
        if not self.waypoints:
            print("No waypoints found to generate trajectory.")
            return None

        q = [[th[i] for th in self.waypoints] for i in range(self.ndof)]
        
        # ensure that then number of points is greater than the degree of the spline (default is 3)
        k = 3 if len(self.waypoints) > 3 else len(self.waypoints)-1

        tck,u = interpolate.splprep(q, k=k, s=0)
        u = np.linspace(0, 1, num=self.nsteps, endpoint=True)

        self.t = np.linspace(0, 1, self.nsteps)
        
        out = interpolate.splev(u,tck)
        self.pos_traj = list(zip(*out))

        out = interpolate.splev(u, tck, der=1)
        self.vel_traj = list(zip(*out))

        # ensure that spline degree is greater than 2 before computing acceleration
        if k > 2:
            out = interpolate.splev(u, tck, der=2)
            self.acc_traj = list(zip(*out))
        else:
            self.acc_traj = [[0.0] * self.ndof for _ in range(self.nsteps)]

        return self.pos_traj
    

class TrapezoidVelocity():
    """
    Trapezoidal velocity profile generator for constant acceleration/deceleration phases.
    """
    
    def __init__(self, trajgen):
        pass



class MultiSegmentTrajectoryGenerator():
    """
    Multi-segment trajectory generator.

    Supports:
    - Piecewise polynomial trajectories (cubic/quintic)
    - B-spline trajectories

    Output:
        t : (N,)
        X : (ndof, 3, N)
    """
    
    def __init__(self, method="quintic",
                 mode="joint",
                 ndof=1
                 ):
        """
        Initialize the trajectory generator with the given configuration.
        """
        self.ndof = ndof
        self.method = method
        self.model = None

        if mode == "joint":
            self.mode = "Joint Space"
            self.labels = [f'axis{i+1}' for i in range(self.ndof)]
        elif mode == "task":
            self.mode = "Task Space"
            self.labels = ['x', 'y', 'z']

    
    def solve(self, waypoints, T):
        """
        Fit the trajectories and solve for the trajectory parameters.

        Args:
            waypoints : (N, ndof)
            T         : segment duration
        """

        wp = np.asarray(waypoints, dtype=float)

        self.waypoints = wp
        self.n_segments = wp.shape[0] - 1
        self.T = T

        # B-Spline case
        if self.method == "bspline":
            self.model = BSpline(ndof=self.ndof)
            self.model.solve(wp)
            return

        # Piecewise polynomial case
        self.segment_models = []
        for i in range(self.n_segments):
            # position constraints at waypoints
            q0 = wp[i]
            qf = wp[i + 1]

            # Velocity continuity at waypoints
            if i == 0: # first segment, start velocity is zero
                qd0 = np.zeros(self.ndof)
            else:
                qd0 = (wp[i] - wp[i - 1]) / self.T

            if i == self.n_segments - 1: # last segment, final velocity is zero
                qdf = np.zeros(self.ndof)
            else:
                qdf = (wp[i + 1] - wp[i]) / self.T

            # select trajectory generation method for the segments
            if self.method == "quintic":
                model = QuinticPolynomial(ndof=self.ndof)
                model.solve(q0, qf, qd0, qdf, None, None, T=self.T)
            elif self.method == "cubic":
                model = CubicPolynomial(ndof=self.ndof)
                model.solve(q0, qf, qd0, qdf, T=self.T)
            self.segment_models.append(model)

    
    def generate(self, nsteps_per_segment=100):
        """
        Generate trajectory.

        Args:
            nsteps_per_segment : number of samples per segment

        Returns:
            t : (N,)
            X : (ndof, 3, N)
        """

        # B-spline case
        if self.method == "bspline":
            self.u, self.X = self.model.generate(nsteps=nsteps_per_segment)
            return
        
        # Piecewise polynomial case
        segments_X = []
        segments_t = []

        for i, model in enumerate(self.segment_models):
            t_, X_ = model.generate(t0=i*self.T, tf=(i+1)*self.T, nsteps=nsteps_per_segment)

            # Avoid duplicate points at boundaries
            if i > 0:
                t_ = t_[1:]
                X_ = X_[:, :, 1:]

            segments_t.append(t_)
            segments_X.append(X_)

        # Concatenate all segments
        self.t = np.concatenate(segments_t)
        self.X = np.concatenate(segments_X, axis=2)

        # return self.t.copy(), self.X.copy()
    

    def plot(self):
        """
        Plot trajectory (position, velocity, acceleration).

        Args:
            t : (N,) time vector
            X : (ndof, 3, N) trajectory array
        """

        fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
        fig.suptitle("Multi-Segment Trajectory", fontsize=16)

        labels = [f'axis{i+1}' for i in range(self.ndof)]
        colors = ['r', 'g', 'b', 'm', 'y']

        for i in range(self.ndof):
            c = colors[i]+'o-'

            axs[0].plot(self.t, self.X[i, 0, :], c, label=labels[i]) # Position
            axs[1].plot(self.t, self.X[i, 1, :], c, label=labels[i]) # Velocity
            axs[2].plot(self.t, self.X[i, 2, :], c, label=labels[i]) # Acceleration

        axs[0].set_ylabel("Position")
        axs[1].set_ylabel("Velocity")
        axs[2].set_ylabel("Acceleration")
        axs[2].set_xlabel("Time (s)")

        for ax in axs:
            ax.grid(True)
            ax.legend()

        plt.tight_layout()
        plt.show()