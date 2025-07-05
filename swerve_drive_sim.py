# swerve_design_sim.py
import numpy as np
import matplotlib.pyplot as plt

# --- Base Dimensions (in Inches) ---
FIELD_SIDE_IN = 144.0
ROBOT_DIM_IN = 12.0
BUMPER_THICKNESS_IN = 1.0

# --- Robot Constants ---
ROBOT_WEIGHT_LBS = 20.0
NUM_DRIVE_MOTORS = 8
WHEEL_DIAMETER_INCHES = 2.5

# --- Environment Constants ---
COEFFICIENT_OF_FRICTION = 1.2
GRAVITY_MS2 = 9.81
INCHES_TO_METERS = 0.0254

# --- Path Distance Calculations (in Inches) ---
total_robot_width_in = ROBOT_DIM_IN + (2 * BUMPER_THICKNESS_IN)
path_side_in = FIELD_SIDE_IN - total_robot_width_in
path_diagonal_in = (FIELD_SIDE_IN - total_robot_width_in) * np.sqrt(2)
arc_radius_in = path_side_in
path_curve_in = (np.pi / 2) * arc_radius_in

# --- Conversions to SI Units (for Physics Calculations) ---
ROBOT_MASS_KG = ROBOT_WEIGHT_LBS * 0.453592
WHEEL_RADIUS_M = (WHEEL_DIAMETER_INCHES * INCHES_TO_METERS) / 2
path_side_m = path_side_in * INCHES_TO_METERS
path_diagonal_m = path_diagonal_in * INCHES_TO_METERS
path_curve_m = path_curve_in * INCHES_TO_METERS

# --- Print Initial Setup ---
print("--- Swerve Design Simulation ---")
print(f"Robot Mass: {ROBOT_MASS_KG:.2f} kg ({ROBOT_WEIGHT_LBS} lbs)")
print(f"Robot Footprint: {total_robot_width_in:.2f}\" x {total_robot_width_in:.2f}\"")
print(f"Coefficient of Friction: {COEFFICIENT_OF_FRICTION}")
print("--------------------------------")

# --- Traction Limit Calculations ---
max_tractive_force_n = ROBOT_MASS_KG * GRAVITY_MS2 * COEFFICIENT_OF_FRICTION
torque_per_motor_before_slip_nm = (max_tractive_force_n * WHEEL_RADIUS_M) / NUM_DRIVE_MOTORS
max_acceleration_ms2 = max_tractive_force_n / ROBOT_MASS_KG

# --- Print Performance Limits ---
print("--- Performance Limits ---")
print(f"Max Tractive Force (Total): {max_tractive_force_n:.2f} N")
print(f"Max Linear Acceleration: {max_acceleration_ms2:.2f} m/s^2")
print(f"Torque per Motor before Slip: {torque_per_motor_before_slip_nm:.4f} Nm")
print("--------------------------")

# --- Drag Race Simulation (Constant Max Acceleration) ---
v_final_side_ms = np.sqrt(2 * max_acceleration_ms2 * path_side_m)
v_final_diagonal_ms = np.sqrt(2 * max_acceleration_ms2 * path_diagonal_m)
v_final_curve_ms = np.sqrt(2 * max_acceleration_ms2 * path_curve_m)

# --- Print Drag Race Results ---
print("--- Drag Race Simulation Results (Achievable Top Speed) ---")
print(f"Side Path ({path_side_in:.2f} in): \t\t{v_final_side_ms:.2f} m/s")
print(f"Diagonal Path ({path_diagonal_in:.2f} in): {v_final_diagonal_ms:.2f} m/s")
print(f"Curved Path ({path_curve_in:.2f} in): \t{v_final_curve_ms:.2f} m/s")
print("---------------------------------------------------------")

# --- Plotting Field Paths ---
plt.style.use('dark_background')
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')

# Field boundary
field_boundary = plt.Rectangle((0, 0), FIELD_SIDE_IN, FIELD_SIDE_IN, facecolor='none', edgecolor='white', lw=2)
ax.add_patch(field_boundary)

# Robot's drivable area
robot_half_width = total_robot_width_in / 2
drivable_area_origin = robot_half_width

# Define path start and end points (center of robot)
start_pos = (robot_half_width, robot_half_width)
end_pos_side = (FIELD_SIDE_IN - robot_half_width, robot_half_width) 
end_pos_diag = (FIELD_SIDE_IN - robot_half_width, FIELD_SIDE_IN - robot_half_width)

# Plot Paths
# 1. Side Path (horizontal)
ax.plot([start_pos[0], end_pos_side[0]], [start_pos[1], end_pos_side[1]], color='cyan', label=f'Side Path ({path_side_in:.1f}")', linestyle='--')

# 2. Diagonal Path
ax.plot([start_pos[0], end_pos_diag[0]], [start_pos[1], end_pos_diag[1]], color='lime', label=f'Diagonal Path ({path_diagonal_in:.1f}")', linestyle='--')

# 3. Curved Path (Quarter circle arc)
arc_center = (robot_half_width, FIELD_SIDE_IN - robot_half_width) # Center moved to top-left of drivable area
theta = np.linspace(3 * np.pi / 2, 2 * np.pi, 100) # 270 to 360 degrees
x_arc = arc_center[0] + arc_radius_in * np.cos(theta)
y_arc = arc_center[1] + arc_radius_in * np.sin(theta)
ax.plot(x_arc, y_arc, color='magenta', label=f'Curved Path ({path_curve_in:.1f}")', linestyle='--')

# Set Plot Options
ax.set_xlim(0, FIELD_SIDE_IN)
ax.set_ylim(0, FIELD_SIDE_IN)
ax.set_xlabel('Field X-Position (inches)')
ax.set_ylabel('Field Y-Position (inches)')
ax.set_title('Top-Down View of Simulated Paths')
ax.legend(loc='upper left')
ax.grid(True, alpha=0.2)
plt.show()