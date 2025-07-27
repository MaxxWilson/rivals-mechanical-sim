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
GEAR_RATIO = 5.0
MOTOR_INERTIA = 0.00000573 # Final estimated value
FLYWHEEL_INERTIA = MOTOR_INERTIA

# --- Analysis Parameters ---
# We're reverting to this more flexible method for defining the analysis window.
SETTLING_THRESHOLD = 0.95 # Clip the analysis when velocity reaches 95% of its peak
STEADY_STATE_POINTS_TO_KEEP = 0 # Keep a few points of the plateau for context
VELOCITY_THRESHOLD = 10 # rev/s