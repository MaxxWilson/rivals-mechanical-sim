# swerve_design_sim.py
import numpy as np

# --- Robot Constants ---
ROBOT_WEIGHT_LBS = 20.0
NUM_DRIVE_MOTORS = 8
WHEEL_DIAMETER_INCHES = 2.5

# --- Environment Constants ---
COEFFICIENT_OF_FRICTION = 1.2
FIELD_SIZE_FT = 12.0
GRAVITY_MS2 = 9.81

# --- Conversions to SI Units ---
ROBOT_MASS_KG = ROBOT_WEIGHT_LBS * 0.453592
FIELD_SIZE_M = FIELD_SIZE_FT * 0.3048
WHEEL_RADIUS_M = (WHEEL_DIAMETER_INCHES * 0.0254) / 2

# --- Print Initial Setup ---
print("--- Swerve Design Simulation ---")
print(f"Robot Mass: {ROBOT_MASS_KG:.2f} kg ({ROBOT_WEIGHT_LBS} lbs)")
print(f"Number of Drive Motors: {NUM_DRIVE_MOTORS}")
print(f"Wheel Diameter: {WHEEL_DIAMETER_INCHES} inches")
print(f"Coefficient of Friction: {COEFFICIENT_OF_FRICTION}")
print(f"Field Size: {FIELD_SIZE_M:.2f}m x {FIELD_SIZE_M:.2f}m")
print("--------------------------------")

# --- Traction Limit Calculations ---

# 1. Calculate the maximum total force the robot can apply before slipping.
max_tractive_force_n = ROBOT_MASS_KG * GRAVITY_MS2 * COEFFICIENT_OF_FRICTION

# 2. Calculate the total torque required at the wheels to generate this force.
total_wheel_torque_nm = max_tractive_force_n * WHEEL_RADIUS_M

# 3. Divide by the number of motors to find the torque required at each wheel axle.
#    This assumes the force is distributed evenly across all drive motors.
torque_per_motor_before_slip_nm = total_wheel_torque_nm / NUM_DRIVE_MOTORS


# --- Print Results ---
print("--- Performance Limits ---")
print(f"Max Tractive Force (Total): {max_tractive_force_n:.2f} N")
print(f"Torque per Motor before Slip: {torque_per_motor_before_slip_nm:.4f} Nm")
print("--------------------------")