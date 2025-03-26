import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from IPython.display import HTML
 
# Define the curve function
def f(x):
    return np.cos(x) * np.exp(x**2)
 
# First derivative (tangent slope)
def df(x):
    return -np.sin(x) * np.exp(x**2) + 2*x*np.cos(x) * np.exp(x**2)
 
# Second derivative (for curvature calculation)
def d2f(x):
    return (-np.cos(x) * np.exp(x**2) - 2*x*np.sin(x) * np.exp(x**2) +
            2*np.cos(x) * np.exp(x**2) + 4*x**2*np.cos(x) * np.exp(x**2))
 
# Compute curvature (κ) and radius of curvature (R)
def curvature(x):
    return np.abs(d2f(x)) / (1 + df(x)**2) ** (3/2)
 
def radius_of_curvature(x):
    kappa = curvature(x)
    return 1 / kappa if kappa != 0 else np.inf
 
# Generate x values for the curve
x_vals = np.linspace(-1, 1, 200)
y_vals = f(x_vals)
 
# Create figure and axis
fig, ax = plt.subplots(figsize=(8, 6))
ax.plot(x_vals, y_vals, 'b', label="Curve")
particle, = ax.plot([], [], 'ro', markersize=6, label="Particle")
tangent_line, = ax.plot([], [], 'r', lw=1.5, label="Tangent")
normal_line, = ax.plot([], [], 'g', lw=1.5, label="Normal")
osc_circle, = ax.plot([], [], 'm', lw=1.2, label="Osculating Circle")
 
ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-1.5, 2)
ax.legend()
ax.set_title("Animation of Tangent, Normal, and Osculating Circle")
 
# Animation function
def update(i):
    x = x_vals[i]
    y = f(x)
   
    # Compute tangent and normal vectors
    slope = df(x)
    dx = 0.2
    dy = slope * dx
    norm_slope = -1/slope if slope != 0 else np.inf
    norm_dx = 0.2 / np.sqrt(1 + norm_slope**2) if norm_slope != np.inf else 0
    norm_dy = norm_slope * norm_dx if norm_slope != np.inf else 0.2
 
    # Compute osculating circle center and radius
    R = radius_of_curvature(x)
    kappa = curvature(x)
    if kappa == 0 or R > 10:  # Avoid too-large circles
        circle_x, circle_y = [], []
    else:
        normal_dir = np.array([-slope, 1]) / np.sqrt(1 + slope**2)
        center_x = x + R * normal_dir[0]
        center_y = y + R * normal_dir[1]
       
        theta = np.linspace(0, 2*np.pi, 100)
        circle_x = center_x + R * np.cos(theta)
        circle_y = center_y + R * np.sin(theta)
 
    # Update elements
    particle.set_data([x], [y])
    tangent_line.set_data([x-dx, x+dx], [y-dy, y+dy])
    normal_line.set_data([x, x+norm_dx], [y, y+norm_dy])
    osc_circle.set_data(circle_x, circle_y)
 
    return particle, tangent_line, normal_line, osc_circle
 
# Create the animation (remove blit for Colab compatibility)
ani = animation.FuncAnimation(fig, update, frames=len(x_vals), interval=50)
 
# Display animation in Google Colab
HTML(ani.to_jshtml())
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from IPython.display import HTML
 
# Define the curve function
def f(x):
    return np.cos(x) * np.exp(x**2)
 
# First derivative (tangent slope)
def df(x):
    return -np.sin(x) * np.exp(x**2) + 2*x*np.cos(x) * np.exp(x**2)
 
# Second derivative (for curvature calculation)
def d2f(x):
    return (-np.cos(x) * np.exp(x**2) - 2*x*np.sin(x) * np.exp(x**2) +
            2*np.cos(x) * np.exp(x**2) + 4*x**2*np.cos(x) * np.exp(x**2))
 
# Compute curvature (κ) and radius of curvature (R)
def curvature(x):
    return np.abs(d2f(x)) / (1 + df(x)**2) ** (3/2)
 
def radius_of_curvature(x):
    kappa = curvature(x)
    return 1 / kappa if kappa != 0 else np.inf
 
# Generate x values for the curve
x_vals = np.linspace(-1, 1, 200)
y_vals = f(x_vals)
 
# Create figure and axis
fig, ax = plt.subplots(figsize=(8, 6))
ax.plot(x_vals, y_vals, 'b', label="Curve")
particle, = ax.plot([], [], 'ro', markersize=6, label="Particle")
tangent_line, = ax.plot([], [], 'r', lw=1.5, label="Tangent")
normal_line, = ax.plot([], [], 'g', lw=1.5, label="Normal")
osc_circle, = ax.plot([], [], 'm', lw=1.2, label="Osculating Circle")
 
ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-1.5, 2)
ax.legend()
ax.set_title("Animation of Tangent, Normal, and Osculating Circle")
 
# Animation function
def update(i):
    x = x_vals[i]
    y = f(x)
   
    # Compute tangent and normal vectors
    slope = df(x)
    dx = 0.2
    dy = slope * dx
    norm_slope = -1/slope if slope != 0 else np.inf
    norm_dx = 0.2 / np.sqrt(1 + norm_slope**2) if norm_slope != np.inf else 0
    norm_dy = norm_slope * norm_dx if norm_slope != np.inf else 0.2
 
    # Compute osculating circle center and radius
    R = radius_of_curvature(x)
    kappa = curvature(x)
    if kappa == 0 or R > 10:  # Avoid too-large circles
        circle_x, circle_y = [], []
    else:
        normal_dir = np.array([-slope, 1]) / np.sqrt(1 + slope**2)
        center_x = x + R * normal_dir[0]
        center_y = y + R * normal_dir[1]
       
        theta = np.linspace(0, 2*np.pi, 100)
        circle_x = center_x + R * np.cos(theta)
        circle_y = center_y + R * np.sin(theta)
 
    # Update elements
    particle.set_data([x], [y])
    tangent_line.set_data([x-dx, x+dx], [y-dy, y+dy])
    normal_line.set_data([x, x+norm_dx], [y, y+norm_dy])
    osc_circle.set_data(circle_x, circle_y)
 
    return particle, tangent_line, normal_line, osc_circle
 
# Create the animation (remove blit for Colab compatibility)
ani = animation.FuncAnimation(fig, update, frames=len(x_vals), interval=50)
 
# Display animation in Google Colab
HTML(ani.to_jshtml())
 
 
 
 
 
 
 