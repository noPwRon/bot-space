# Instructions & Learning Resources

## Project Goal
Build a robot simulator that:
1. Accepts a Denavit-Hartenberg (DH) table of any size (any number of joints)
2. Computes forward kinematics using homogeneous transformation matrices
3. Derives equations of motion using Lagrangian mechanics
4. Numerically integrates the equations of motion to simulate robot movement
5. Visualizes the robot's motion over time
6. Solves inverse kinematics to follow a given end-effector path
7. Generates paths from real-world scenarios or mathematical functions (e.g. circles, spirals)
8. Outputs simulation graphs — torque, position, velocity, and other quantities over time
9. Supports real-world load scenarios (e.g. picking up and moving a block)
10. Assists with motor sizing by reporting peak joint torques from a simulation

---

## Recommended Learning Order

### Stage 1 — Python Foundations
Before writing any robotics code, get comfortable with the Python tools you will use constantly.

| Topic | Resource |
|---|---|
| Python basics (data types, loops, functions) | https://docs.python.org/3/tutorial/ |
| NumPy (arrays, matrix math) | https://numpy.org/doc/stable/user/quickstart.html |
| Reading YAML files in Python | https://pyyaml.org/wiki/PyYAMLDocumentation |
| Python classes and OOP | https://docs.python.org/3/tutorial/classes.html |

### Stage 2 — Linear Algebra for Robotics
DH tables and transforms are built entirely on linear algebra.

| Topic | Resource |
|---|---|
| Vectors and matrices (intuition) | https://www.3blue1brown.com/topics/linear-algebra (3Blue1Brown — free videos) |
| Rotation matrices and homogeneous coordinates | "Introduction to Robotics" — Craig, Ch. 1-2 |
| NumPy linear algebra reference | https://numpy.org/doc/stable/reference/routines.linalg.html |

### Stage 3 — Denavit-Hartenberg Convention
The DH convention is a standard way to describe the geometry of a robot arm joint by joint.

| Topic | Resource |
|---|---|
| DH parameters explained | "Introduction to Robotics" — Craig, Ch. 3 |
| Video walkthrough of DH tables | https://www.youtube.com/watch?v=rA9tm0gTln8 (Robotics Academy) |
| Interactive DH visualizer | https://www.rosroboticslearning.com/forward-kinematics |

**Key concepts to understand before coding:**
- The four DH parameters: `a`, `d`, `alpha`, `theta`
- How each parameter describes a translation or rotation between two joint frames
- How chaining T matrices gives you the position and orientation of the end-effector

### Stage 4 — Lagrangian Mechanics
Lagrangian mechanics gives us the equations of motion from energy, rather than forces.

| Topic | Resource |
|---|---|
| Lagrangian mechanics intuition | https://www.youtube.com/watch?v=KpLno70oYHE (Physics videos by Eugene Khutoryansky) |
| Formal derivation | "Classical Mechanics" — Goldstein, Ch. 1-2 |
| Lagrangian robotics dynamics | "Introduction to Robotics" — Craig, Ch. 6 |
| SymPy for symbolic math in Python | https://docs.sympy.org/latest/tutorials/intro-tutorial/index.html |

**Key concepts to understand before coding:**
- Kinetic energy T and potential energy V for a rigid body
- The Lagrangian: L = T - V
- The Euler-Lagrange equation and how it produces equations of motion
- Generalised coordinates (joint angles as your variables)

### Stage 5 — Numerical Simulation
Once you have symbolic equations of motion, you need to integrate them numerically.

| Topic | Resource |
|---|---|
| Ordinary differential equations (ODEs) | https://tutorial.math.lamar.edu/Classes/DE/DE.aspx |
| SciPy ODE solver (solve_ivp) | https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html |

### Stage 6 — Visualization
Plotting and animating the robot's motion.

| Topic | Resource |
|---|---|
| Matplotlib basics | https://matplotlib.org/stable/tutorials/index.html |
| Matplotlib 3D animation | https://matplotlib.org/stable/gallery/animation/index.html |

### Stage 7 — Inverse Kinematics
Given a desired end-effector position, solve for the joint angles that achieve it.

| Topic | Resource |
|---|---|
| Inverse kinematics overview | "Introduction to Robotics" — Craig, Ch. 4 |
| Jacobian-based IK (numerical) | "Robot Modeling and Control" — Spong, Ch. 6 |
| scipy.optimize for numerical IK | https://docs.scipy.org/doc/scipy/reference/optimize.html |

**Key concepts to understand before coding:**
- The difference between analytical IK (closed-form equations) and numerical IK (iterative solvers)
- The Jacobian pseudo-inverse method for redundant robots
- Singularities — configurations where IK has no unique solution

### Stage 8 — Path Generation
Defining end-effector trajectories in Cartesian space.

| Topic | Resource |
|---|---|
| Trajectory planning | "Introduction to Robotics" — Craig, Ch. 7 |
| Parametric curves (circles, spirals) | https://mathworld.wolfram.com/ParametricEquations.html |

**Key concepts:**
- A path is a sequence of end-effector poses in 3D space
- IK converts each pose in the path into joint angles
- The simulator then drives the robot along those joint angle sequences

### Stage 9 — Simulation Output and Analysis
Extracting and plotting meaningful data from simulation results.

| Topic | Resource |
|---|---|
| Matplotlib subplots | https://matplotlib.org/stable/gallery/subplots_axes_and_figures/subplots_demo.html |

**Quantities to plot:**
- Joint angles (position) over time
- Joint velocities over time
- Joint torques over time — computed from the equations of motion
- End-effector position over time (extracted from forward kinematics at each timestep)

### Stage 10 — Real-World Scenarios
Extending the simulator to handle external loads and practical use cases.

**Key additions:**
- External force/torque at the end-effector (e.g. holding a block) — adds a term to the equations of motion
- Motor sizing — run a task simulation and report peak torque per joint
- Pick-and-place — combine a path, IK, and a load change mid-simulation

---

## File Roadmap
Work through the files roughly in this order:

1. `configs/robots/example_6dof.yaml` — define your first DH table by hand ✓
2. `src/kinematics/dh_table.py` — load and validate the YAML DH table ✓
3. `src/kinematics/transforms.py` — build T matrices and chain them (forward kinematics) ✓
4. `src/dynamics/lagrangian.py` — symbolic kinetic and potential energy ✓
5. `src/dynamics/solver.py` — derive and solve equations of motion ✓
6. `src/simulation/simulator.py` — numerical integration loop ✓
7. `src/visualization/renderer.py` — plot and animate results ✓
8. `tests/test_kinematics.py` — verify your transforms are correct ✓
9. `tests/test_dynamics.py` — verify your equations of motion ✓
10. `src/kinematics/inverse_kinematics.py` — solve for joint angles given an end-effector pose
11. `src/planning/path_generator.py` — generate end-effector paths (lines, circles, custom)
12. `src/visualization/plotter.py` — plot torque, position, velocity over time
13. `src/simulation/scenarios.py` — real-world scenarios (pick and place, external loads)

---

## Recommended Textbook
**"Introduction to Robotics: Mechanics and Control"** — John J. Craig (3rd edition)
This is the standard reference for DH tables and robot kinematics/dynamics.
It is the primary textbook this project is structured around.
