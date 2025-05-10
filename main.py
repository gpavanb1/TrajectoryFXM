import numpy as np
import matplotlib.pyplot as plt
import pickle

from splitfxm.domain import Domain
from splitfxm.simulation import Simulation
from splitfxm.schemes import default_scheme
from splitfxm.visualize import draw

from trajectory.equation import TrajectoryEquation
from trajectory.model import TrajectoryModel

# ------------------------------------------------------------------------------
# Problem parameters
# ------------------------------------------------------------------------------

T = 1e5         # Thrust (N)
L = 230910.1    # Domain length (m)
v0 = 100.0      # Initial velocity (m/s)
g = 9.81        # Gravitational acceleration (m/s²)
m = 100.0       # Mass (kg)

# ------------------------------------------------------------------------------
# Create the model and domain
# ------------------------------------------------------------------------------
method = "FDM"
model = TrajectoryModel(T=T, L=L, g=g, m=m)

# Domain: 80 interior cells, 1 left + 1 right boundary cell, 2 variables (y, v)
d = Domain.from_size(80, 1, 1,
                     ["y", "v"], xmin=0.0, xmax=L)

# Boundary conditions
bcs = {
    "y": {
        "left": {"dirichlet": 0.0},
        "right": {"dirichlet": 0.0}
    },
    "v": {
        "left": {"dirichlet": v0},
        "right": "outflow"
    }
}

# Initialize the simulation
s = Simulation(d, model, ics={}, bcs=bcs, scheme=default_scheme(method))

# ------------------------------------------------------------------------------
# Load initial guess from pickle files (x, y, v grids)
# ------------------------------------------------------------------------------

with open("./seed/x_grid.pkl", "rb") as f:
    x_grid = pickle.load(f)

with open("./seed/y_grid.pkl", "rb") as f:
    y_grid = pickle.load(f)

with open("./seed/v_grid.pkl", "rb") as f:
    v_grid = pickle.load(f)

# Interpolation functions for initializing y and v


def y_interp(x): return np.interp(x, x_grid, y_grid)
def v_interp(x): return np.interp(x, x_grid, v_grid)

# ------------------------------------------------------------------------------
# Set initial values in the domain using the interpolated profiles
# ------------------------------------------------------------------------------


for cell in d.interior():
    x = cell.x()
    cell.set_value(0, y_interp(x))  # Initialize y
    cell.set_value(1, v_interp(x))  # Initialize v

# ------------------------------------------------------------------------------
# Run the solver to steady state
# ------------------------------------------------------------------------------

num_iter = s.steady_state(split=True, split_locs=[1])

# ------------------------------------------------------------------------------
# Visualize the result
# ------------------------------------------------------------------------------

x_vals = [c.x()/1e3 for c in d.cells()]
values = [c.values() for c in d.cells()]
y_vals = [v[0] for v in values]
v_vals = [v[1] for v in values]

# Plot y vs x
plt.figure()
plt.plot(x_vals, y_vals)
plt.xlabel("x (km)")
plt.ylabel("y (m)")
plt.title("Trajectory: y vs x")
plt.show()

# Plot v vs x
plt.figure()
plt.plot(x_vals, v_vals)
plt.xlabel("x (km)")
plt.ylabel("v (m/s)")
plt.title("Velocity Profile: v vs x")
plt.show()
