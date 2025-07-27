# config.py
"""
Configuration file for the motor dyno analysis script.
"""

# --- Column Mapping ---
COLUMN_MAP = {
    "time": "time",
    "velocity": "ODrive 0 Velocity",
    "current": "ODrive 0 Current"
}

# --- Physics & Hardware Constants ---
MOTOR_KV = 1450
GEAR_RATIO = 14.0
MOTOR_INERTIA = 0.00000573
FLYWHEEL_INERTIA = 0.000

# --- Analysis Parameters ---
SETTLING_THRESHOLD = 0.95
STEADY_STATE_POINTS_TO_KEEP = 3
VELOCITY_THRESHOLD = 10 # rev/s