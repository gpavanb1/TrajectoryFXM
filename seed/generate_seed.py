# ------------------------------------------------------------------------------
# Simulate a 2D projectile trajectory with thrust and drag using solve_ivp
# ------------------------------------------------------------------------------

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin
from scipy.integrate import solve_ivp
import pickle

# ------------------------------------------------------------------------------
# ODE system: 2D trajectory under thrust, drag, and gravity
# ------------------------------------------------------------------------------


def trajectory_ivp(t, u, params):
    """
    ODE system for 2D projectile motion with thrust and drag.

    Parameters:
        t      : time (s)
        u      : state vector [x, y, v, theta]
        params : physical parameters (tuple)

    Returns:
        derivatives [dx/dt, dy/dt, dv/dt, dtheta/dt]
    """
    x, y, v, theta = u
    rho, Cd, A, T_burn, burn_time, m, g = params

    # Constant thrust (assumed here for simplicity)
    T = T_burn

    # Aerodynamic drag
    D = 0.5 * rho * Cd * A * v**2

    # Translational motion
    dxdt = v * cos(theta)
    dydt = v * sin(theta)

    # Speed derivative
    dvdt = (T - D) / m - g * sin(theta)

    # Flight-path angle derivative (prevent division by zero)
    dthetadt = ((T - D) / m * (-sin(theta)) - g * cos(theta)) / (v + 1e-8)

    return [dxdt, dydt, dvdt, dthetadt]

# ------------------------------------------------------------------------------
# Parameters and initial conditions
# ------------------------------------------------------------------------------


# Physical parameters
rho = 1.0       # air density (kg/m³)
Cd = 0.3       # drag coefficient
A = 1e-2      # cross-sectional area (m²)
T_burn = 1e5       # thrust (N)
burn_time = 10.0    # burn duration (s)
m = 100.0     # mass (kg)
g = 9.81      # gravitational acceleration (m/s²)

params = (rho, Cd, A, T_burn, burn_time, m, g)

# Initial state: [x, y, v, theta]
v_launch = 100.0
launch_angle = np.deg2rad(80.0)
u0 = [0.0, 0.0, v_launch, launch_angle]

# ------------------------------------------------------------------------------
# Event to stop integration when projectile hits ground
# ------------------------------------------------------------------------------


def hit_ground(t, u):
    """Event function to stop integration when y = 0 (descending)."""
    if t < 1e-3:
        return 1.0  # ignore very early crossings
    return u[1]


hit_ground.terminal = True
hit_ground.direction = -1  # only detect downward crossings

# ------------------------------------------------------------------------------
# Integrate ODEs using solve_ivp
# ------------------------------------------------------------------------------

sol = solve_ivp(
    fun=lambda t, u: trajectory_ivp(t, u, params),
    t_span=(0, 100),
    y0=u0,
    events=hit_ground,
    max_step=0.1
)

# ------------------------------------------------------------------------------
# Extract and save results
# ------------------------------------------------------------------------------

x_vals = sol.y[0]
y_vals = sol.y[1]
v_vals = sol.y[2]

# Save to pickle files for use in BVP initialization
with open("x_grid.pkl", "wb") as f:
    pickle.dump(x_vals, f)

with open("y_grid.pkl", "wb") as f:
    pickle.dump(y_vals, f)

with open("v_grid.pkl", "wb") as f:
    pickle.dump(v_vals, f)

# ------------------------------------------------------------------------------
# Plot results
# ------------------------------------------------------------------------------

plt.figure()
plt.plot(x_vals, y_vals)
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("Trajectory: y vs x")
plt.grid(True)
plt.show()

plt.figure()
plt.plot(x_vals, v_vals)
plt.xlabel("x (m)")
plt.ylabel("v (m/s)")
plt.title("Velocity Profile: v vs x")
plt.grid(True)
plt.show()

# ------------------------------------------------------------------------------
# Print key outcomes
# ------------------------------------------------------------------------------

t_impact = sol.t_events[0][0]
x_impact = sol.y_events[0][0][0]
print(f"Flight time ≈ {t_impact:.2f} s, range ≈ {x_impact:.1f} m")
