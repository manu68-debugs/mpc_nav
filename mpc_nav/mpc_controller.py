import copy
import math
import os
import csv
import numpy as np
import scipy.interpolate as interpolate
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs

import casadi as ca

class MPCTrackerNode(Node):
    def __init__(self):
        super().__init__('mpc_tracker_node')
        
        # ROS Parameters
        self.declare_parameter('path_file', 'mpc_nav/waypoints/waypoint.csv') # Path to waypoints relative to workspace
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('a_lat_max', 0.5) # max lateral acceleration
        self.declare_parameter('a_lon_max', 0.3) # max longitudinal acceleration
        self.declare_parameter('dt', 0.1)        # MPC discretization time
        self.declare_parameter('N', 15)          # MPC horizon
        self.declare_parameter('d_safe', 0.7)    # Obstacle avoidance boundary

        # Fetch params
        self.v_max = self.get_parameter('v_max').value
        self.a_lat = self.get_parameter('a_lat_max').value
        self.a_lon = self.get_parameter('a_lon_max').value
        self.dt = self.get_parameter('dt').value
        self.N = self.get_parameter('N').value
        self.d_safe = self.get_parameter('d_safe').value
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_viz_pub = self.create_publisher(Path, 'reference_trajectory', 10)
        self.horizon_viz_pub = self.create_publisher(MarkerArray, 'mpc_horizon_markers', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Internal State
        self.laser_points = []
        self.robot_pose = None
        self.goal_reached = False
        self.start_time = None
        
        # 1 & 2. Execute Path Smoothing and Velocity Profiling on startup
        self.init_spline_and_physics()
        
        # 5. Initialize the CasADi MPC solver
        self.init_mpc_solver()
        
        # Start Control Loop
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('MPC tracker initialized.')

    def init_spline_and_physics(self):
        """ Spline fitting and velocity profile generation """
        path_file = self.get_parameter('path_file').value
        
        # Fallback to local workspace searching
        if not os.path.exists(path_file):
            path_file = os.path.join(os.getcwd(), 'src', 'mpc_nav', 'waypoints', 'waypoint.csv')

        waypoints = []
        with open(path_file, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    try:
                        waypoints.append([float(row[0]), float(row[1])])
                    except ValueError:
                        pass # Skip header row
        waypoints = np.array(waypoints)
        
        # Chord length parametrization
        diffs = np.diff(waypoints, axis=0)
        dists = np.linalg.norm(diffs, axis=1)
        self.s_raw = np.concatenate(([0.0], np.cumsum(dists)))
        
        # Fit Cubic Splines for X and Y respect to arc-length s
        self.cs_x = interpolate.CubicSpline(self.s_raw, waypoints[:, 0], bc_type='natural')
        self.cs_y = interpolate.CubicSpline(self.s_raw, waypoints[:, 1], bc_type='natural')
        
        self.cs_x_d = self.cs_x.derivative(1)
        self.cs_x_dd = self.cs_x.derivative(2)
        self.cs_y_d = self.cs_y.derivative(1)
        self.cs_y_dd = self.cs_y.derivative(2)
        
        # Generate dense profile table
        self.s_dense = np.arange(0, self.s_raw[-1], 0.05)
        self.x_dense = self.cs_x(self.s_dense)
        self.y_dense = self.cs_y(self.s_dense)
        
        xp = self.cs_x_d(self.s_dense)
        yp = self.cs_y_d(self.s_dense)
        xpp = self.cs_x_dd(self.s_dense)
        ypp = self.cs_y_dd(self.s_dense)
        
        # Curvature formula
        self.kappa = (xp * ypp - yp * xpp) / (xp**2 + yp**2)**1.5
        
        # Lateral physics constraint: v = sqrt(a_lat / kappa)
        self.v_profile = np.zeros_like(self.s_dense)
        for i in range(len(self.s_dense)):
            k = abs(self.kappa[i])
            if k < 1e-4:
                self.v_profile[i] = self.v_max
            else:
                self.v_profile[i] = min(self.v_max, math.sqrt(self.a_lat / k))
                
        # Apply longitudinal acceleration limits
        ds = 0.05
        # Backward sweep
        self.v_profile[-1] = 0.0 # Must stop at end
        for i in range(len(self.s_dense)-2, -1, -1):
            self.v_profile[i] = min(self.v_profile[i], math.sqrt(self.v_profile[i+1]**2 + 2 * self.a_lon * ds))
        # Forward sweep
        for i in range(1, len(self.s_dense)):
            self.v_profile[i] = min(self.v_profile[i], math.sqrt(self.v_profile[i-1]**2 + 2 * self.a_lon * ds))
            
        # KDTree for rapid closest-point projection
        self.path_tree = KDTree(np.vstack((self.x_dense, self.y_dense)).T)
        self.max_s = self.s_dense[-1]
        
        # Publish static dense path array
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(self.s_dense)):
            p = PoseStamped()
            p.pose.position.x = self.x_dense[i]
            p.pose.position.y = self.y_dense[i]
            # calculate tangent angle for visual
            th = math.atan2(yp[i], xp[i])
            p.pose.orientation.z = math.sin(th/2)
            p.pose.orientation.w = math.cos(th/2)
            msg.poses.append(p)
        self.path_viz_pub.publish(msg)

    def init_mpc_solver(self):
        """ CasADi IPOPT setup for optimal tracking """
        # State: x, y, theta
        self.opti = ca.Opti()
        
        self.x_var = self.opti.variable(3, self.N+1) # [x, y, theta]
        self.u_var = self.opti.variable(2, self.N)   # [v, w]
        
        # Parameters (The reference trajectory we will pass in every loop)
        self.x_ref = self.opti.parameter(3, self.N+1) 
        self.v_ref = self.opti.parameter(1, self.N)
        
        self.x0 = self.opti.parameter(3) # Initial condition
        
        # Kinematic Bicycle / Unicycle Model constraints
        self.opti.subject_to(self.x_var[:, 0] == self.x0)
        for k in range(self.N):
            x_next = self.x_var[0, k] + self.u_var[0, k] * ca.cos(self.x_var[2, k]) * self.dt
            y_next = self.x_var[1, k] + self.u_var[0, k] * ca.sin(self.x_var[2, k]) * self.dt
            th_next = self.x_var[2, k] + self.u_var[1, k] * self.dt
            self.opti.subject_to(self.x_var[0, k+1] == x_next)
            self.opti.subject_to(self.x_var[1, k+1] == y_next)
            self.opti.subject_to(self.x_var[2, k+1] == th_next)
            
        # Cost Function
        Q_x = 20.0
        Q_y = 20.0
        Q_th = 2.5
        R_v = 1.0 # Control efforts
        R_w = 1.0
        R_dv = 5.0 # Smoothness 
        R_dw = 2.0
        
        cost = 0
        for k in range(self.N):
            # Positional Error tracking reference trajectory
            cost += Q_x * (self.x_var[0, k] - self.x_ref[0, k])**2
            cost += Q_y * (self.x_var[1, k] - self.x_ref[1, k])**2
            # Use cosine to inherently bypass 2*pi wrap-around singularities
            cost += (Q_th * 2.0) * (1.0 - ca.cos(self.x_var[2, k] - self.x_ref[2, k]))
            
            # Control effort
            cost += R_v * (self.u_var[0, k] - self.v_ref[0, k])**2
            cost += R_w * (self.u_var[1, k])**2
            
            # Control smoothness (derivatives)
            if k > 0:
                cost += R_dv * (self.u_var[0, k] - self.u_var[0, k-1])**2
                cost += R_dw * (self.u_var[1, k] - self.u_var[1, k-1])**2
        
        # Final terminal cost heavily weighted
        cost += 50.0 * (self.x_var[0, self.N] - self.x_ref[0, self.N])**2
        cost += 50.0 * (self.x_var[1, self.N] - self.x_ref[1, self.N])**2
        cost += 50.0 * (1.0 - ca.cos(self.x_var[2, self.N] - self.x_ref[2, self.N]))
        
        self.opti.minimize(cost)
        
        # Bounds constraints (Prevent driving backward to avoid erratic flipping)
        self.opti.subject_to(self.opti.bounded(0.0, self.u_var[0, :], self.v_max))
        self.opti.subject_to(self.opti.bounded(-1.5, self.u_var[1, :], 1.5))
        
        # Solver specifics
        p_opts = {"print_time": False}
        s_opts = {"print_level": 0, "max_iter": 40}
        self.opti.solver('ipopt', p_opts, s_opts)

    def scan_callback(self, msg):
        """ Store valid LiDAR constraints """
        # Only take points within +/- 60 degrees (in front)
        points = []
        angle = msg.angle_min
        
        # Simple extraction
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Normalize angle to [-pi, pi] to properly process right-side points (5.24 rad -> -1.04 rad)
                norm_angle = (angle + math.pi) % (2 * math.pi) - math.pi
                if -1.04 < norm_angle < 1.04: # Approx 60 deg
                    # Convert to local coordinates (base_footprint)
                    lx = r * math.cos(angle)
                    ly = r * math.sin(angle)
                    points.append((lx, ly))
            angle += msg.angle_increment
            
        self.laser_points = points

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # Quaternion to Euler yaw
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw, t.transform.rotation
        except Exception as e:
            self.get_logger().warn(f'TF Error: {e}', throttle_duration_sec=1.0)
            return None

    def control_loop(self):
        now = self.get_clock().now()
        if self.start_time is None:
            if now.nanoseconds == 0:
                return # Wait for Gazebo clock to begin publishing
            self.start_time = now
            
        if (now - self.start_time).nanoseconds < 5e9:
            self.get_logger().info('Waiting 5 seconds for simulation to stabilize...', throttle_duration_sec=1.0)
            return
            
        self.get_logger().info('Control loop ticking...', throttle_duration_sec=2.0)
        
        if self.goal_reached:
            return
            
        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn('TF pose is None, waiting for TF...', throttle_duration_sec=2.0)
            return
        rx, ry, rth, rq = pose
        
        # Check for imminent collision
        min_dist = float('inf')
        for (lx, ly) in self.laser_points:
            d = math.hypot(lx, ly)
            if d < min_dist:
                min_dist = d
        
        if min_dist < 0.20:
            self.get_logger().warn('Obstacle too close, applying emergency stop.', throttle_duration_sec=1.0)
            self.cmd_pub.publish(Twist())
            return
        
        # Check termination (distance to final s_dense point)
        dist_to_end = math.hypot(rx - self.x_dense[-1], ry - self.y_dense[-1])
        if dist_to_end < 0.2:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Goal Reached via MPC!')
            self.goal_reached = True
            return

        # Generate time-integrated lookahead trajectory
        _, idx = self.path_tree.query([rx, ry])
        s_0 = self.s_dense[idx]
        
        horizon_x = np.zeros(self.N+1)
        horizon_y = np.zeros(self.N+1)
        horizon_th = np.zeros(self.N+1)
        horizon_v = np.zeros(self.N)
        
        s_curr = s_0
        for k in range(self.N):
            # Fetch dense matching state for s_curr
            # Interpolate to strictly adhere to the continuous spline logic
            x_k = self.cs_x(s_curr)
            y_k = self.cs_y(s_curr)
            xp_k = self.cs_x_d(s_curr)
            yp_k = self.cs_y_d(s_curr)
            th_k = math.atan2(yp_k, xp_k)
            
            # Fetch mapped velocity (nearest index interpolation)
            v_idx = min(int(s_curr / 0.05), len(self.v_profile)-1)
            v_k = self.v_profile[v_idx]
            
            # Predict next s
            s_curr = min(self.max_s, s_curr + v_k * self.dt)
            
            horizon_x[k] = x_k
            horizon_y[k] = y_k
            horizon_th[k] = th_k
            horizon_v[k] = v_k
            
        # End point
        horizon_x[self.N] = self.cs_x(s_curr)
        horizon_y[self.N] = self.cs_y(s_curr)
        horizon_th[self.N] = math.atan2(self.cs_y_d(s_curr), self.cs_x_d(s_curr))

        # Trajectory Deformation (Obstacle Avoidance)
        # Transform lidar points (local) to odom (global) to check against global horizon
        global_obs = []
        for (lx, ly) in self.laser_points:
            gx = rx + lx * math.cos(rth) - ly * math.sin(rth)
            gy = ry + lx * math.sin(rth) + ly * math.cos(rth)
            global_obs.append(np.array([gx, gy]))
            
        if global_obs:
            obs_tree = KDTree(global_obs)
            for k in range(1, self.N+1):
                hk = np.array([horizon_x[k], horizon_y[k]])
                dist, o_idx = obs_tree.query(hk)
                if dist < self.d_safe:
                    # Deform trajectory laterally!
                    o_pt = global_obs[o_idx]
                    
                    # 1. Compute purely lateral normal vector from the tangent
                    Nx = -math.sin(horizon_th[k])
                    Ny = math.cos(horizon_th[k])
                    N = np.array([Nx, Ny])
                    
                    # 2. Determine which side the obstacle is on
                    V = hk - o_pt
                    if np.dot(V, N) >= 0:
                        normal = N  # Obstacle on the right, push path to the left
                    else:
                        normal = -N # Obstacle on the left, push path to the right
                        
                    # 3. Apply the lateral push
                    push = (self.d_safe - dist) * 1.5 
                    hk_new = hk + normal * push
                    horizon_x[k] = hk_new[0]
                    horizon_y[k] = hk_new[1]
                    
                    # Velocity dampen for danger points
                    if k < self.N:
                        vel_scaling = max(0.2, dist / self.d_safe)
                        horizon_v[k] *= vel_scaling

            # Re-smooth the tangent headings so kinematic track can curve
            for k in range(1, self.N):
                dx = horizon_x[k+1] - horizon_x[k]
                dy = horizon_y[k+1] - horizon_y[k]
                horizon_th[k] = math.atan2(dy, dx)
            # Propagate back to current step and terminal step
            horizon_th[0] = math.atan2(horizon_y[1]-horizon_y[0], horizon_x[1]-horizon_x[0])
            horizon_th[self.N] = horizon_th[self.N-1]

        self.publish_horizon_markers(horizon_x, horizon_y)

        # ====================================================
        # Solve MPC
        # ====================================================
        self.opti.set_value(self.x0, [rx, ry, rth])
        self.opti.set_value(self.x_ref, np.vstack((horizon_x, horizon_y, horizon_th)))
        self.opti.set_value(self.v_ref, horizon_v.reshape(1, self.N))
        
        try:
            sol = self.opti.solve()
            u_opt = sol.value(self.u_var)
            v_cmd = u_opt[0, 0]
            w_cmd = u_opt[1, 0]
            
            # Hot start next step for performance
            self.opti.set_initial(self.x_var, sol.value(self.x_var))
            self.opti.set_initial(self.u_var, sol.value(self.u_var))
            
        except RuntimeError:
            self.get_logger().warn('IPOPT solve failed! Falling back to backup.')
            # Extract suboptimal solution if it failed, or command 0
            u_fallback = self.opti.debug.value(self.u_var)
            v_cmd = u_fallback[0, 0] if math.isfinite(u_fallback[0, 0]) else 0.0
            w_cmd = u_fallback[1, 0] if math.isfinite(u_fallback[1, 0]) else 0.0
            
        # Command Robot
        t = Twist()
        t.linear.x = float(v_cmd)
        t.angular.z = float(w_cmd)
        self.cmd_pub.publish(t)
        self.get_logger().info(f'Published cmd_vel -> v: {v_cmd:.2f}, w: {w_cmd:.2f}', throttle_duration_sec=2.0)

    def publish_horizon_markers(self, x_arr, y_arr):
        """ Publish markers indicating the reference horizon """
        mrk_arr = MarkerArray()
        for i in range(len(x_arr)):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "horizon"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x_arr[i])
            m.pose.position.y = float(y_arr[i])
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0 # Green dots
            m.lifetime.sec = 0
            m.lifetime.nanosec = int(self.dt * 1e9)
            mrk_arr.markers.append(m)
        self.horizon_viz_pub.publish(mrk_arr)

def main(args=None):
    rclpy.init(args=args)
    node = MPCTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
