import numpy as np
from scipy.interpolate import CubicSpline

# Reference: parametric curves — https://mathworld.wolfram.com/ParametricEquations.html
# Reference: scipy spline interpolation — https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html
# Reference: Craig, "Introduction to Robotics", Ch. 7 (trajectory planning)

# A "path" in this module is a callable: f(t) -> 4x4 numpy array (a pose in the base frame).
# t is a float in [0, 1], where 0 is the start and 1 is the end of the path.
# The 4x4 matrix is a homogeneous transform: top-left 3x3 is orientation, top-right 3x1 is position.

# A "sampler" takes a path callable and a number of steps, and returns a list of poses.
# This separates *what the path is* from *how finely we discretize it*.

# --- Section 0: Helpers ---

def make_function(fx, fy, fz):
    # Make a T matrix at the position x(t),y(t),z(t)
    T = np.eye(4)
    T[:3, 3] = [fx, fy, fz]
    return T


# --- Section 1: Sampler ---

def sample_path(path_fn, n_steps):
    # Call path_fn at n_steps evenly spaced values of t between 0 and 1.
    # Return the results as a list of 4x4 numpy arrays.
    # Hint: numpy has a function that generates evenly spaced values over an interval.

    t = np.linspace(0,1,n_steps)
    t_list = []

    for i in t:
        t_list.append(path_fn(i))

    return t_list


# --- Section 2: Spline path (mode 1 — interpolate through user-provided points) ---

# A spline is a smooth curve that passes exactly through a given set of waypoints.
# You provide a list of (x, y, z) positions; the spline fills in the curve between them.
# Each axis (x, y, z) is interpolated independently as a function of t.
# Reference: CubicSpline from scipy.interpolate does this for you.

def make_spline_path(waypoints):
    # waypoints: a list of (x, y, z) tuples, at least 2 points.
    # t values for each waypoint are evenly spaced in [0, 1].
    # Fit a separate cubic spline for x, y, and z as functions of t.
    # Return a callable f(t) that:
    #   - evaluates all three splines at t to get a position
    #   - builds and returns a 4x4 pose matrix with that position and identity orientation
    # Note: a Python function defined inside another function can "remember" variables
    # from the outer scope — this is called a closure. Look up "Python closures" if unfamiliar.

    t_points = np.linspace(0,1, len(waypoints))

    x_points = [wp[0] for wp in waypoints]
    y_points = [wp[1] for wp in waypoints]
    z_points = [wp[2] for wp in waypoints]


    fx = CubicSpline(t_points, x_points)
    fy = CubicSpline(t_points, y_points)
    fz = CubicSpline(t_points, z_points)

    # TODO: make_function is called once here at definition time, not wrapped in a closure.
    # This means spline_function ends up as a T matrix (the result), not a callable.
    # Fix: define a def f(t): that evaluates fx(t), fy(t), fz(t) and calls make_function
    # with those three float values. Return f, not the result of calling make_function.
    spline_function = make_function(fx, fy, fz, t)

    return spline_function


# --- Section 3: Geometric path (mode 2 — math-defined shapes) ---

# Each function below returns a path callable for a specific shape.
# All shapes are parameterized by t in [0, 1].

def make_line_path(start, end):
    # start, end: (x, y, z) tuples for the two endpoints.
    # At t=0 the pose should be at start; at t=1 it should be at end.
    # Linear interpolation between two points: p(t) = start + t * (end - start)
    # Return a callable f(t) -> 4x4 pose matrix with identity orientation.

    x_points = [start[0], end[0]]
    y_points = [start[1], end[1]]
    z_points = [start[2], end[2]]
    t = [0,1]

    # TODO: fx, fy, fz are not defined — x_points/y_points/z_points were extracted but never used.
    # For a line you don't need CubicSpline at all. The hint says: p(t) = start + t * (end - start).
    # Fix: define a def f(t): that computes x, y, z directly from that formula using start and end,
    # then calls make_function(x, y, z). The x_points/y_points/z_points lists can be removed entirely.
    line_function = make_function(fx, fy, fz, t)

    return line_function


def make_circle_path(center, radius, normal, start_angle=0):
    # center: (x, y, z) of the circle's center
    # radius: float
    # normal: (x, y, z) unit vector perpendicular to the plane of the circle
    #         e.g. (0, 0, 1) means the circle lies flat in the XY plane
    # start_angle: where on the circle to begin, in radians
    # At t=0 the pose is at start_angle; at t=1 it has completed one full revolution (2*pi).
    # Hint: to place a circle in an arbitrary plane defined by `normal`, you need two
    # orthogonal vectors that both lie in that plane — look up "basis vectors from normal".
    # Return a callable f(t) -> 4x4 pose matrix with identity orientation.
    pass


def make_spiral_path(center, radius_start, radius_end, height, normal, n_turns):
    # A spiral that grows in radius and climbs in height over t in [0, 1].
    # center: (x, y, z) of the spiral's base center
    # radius_start, radius_end: radius at t=0 and t=1
    # height: total vertical distance traveled along the normal axis
    # normal: (x, y, z) unit vector for the axis the spiral climbs along
    # n_turns: how many full revolutions the spiral makes
    # Hint: combine the circle logic with a linearly growing radius and a climbing offset.
    # Return a callable f(t) -> 4x4 pose matrix with identity orientation.
    pass


# --- Section 4: Sensor path (mode 3 — stub for future implementation) ---

# Sensor-driven paths update based on real-time input (e.g. a camera or force sensor).
# The interface is the same — a callable f(t) -> 4x4 pose — but the implementation
# will read from an external source rather than computing from math.

def make_sensor_path():
    # For now, raise NotImplementedError with a descriptive message.
    # When sensor input is available, this function will accept a sensor handle or callback.
    pass
