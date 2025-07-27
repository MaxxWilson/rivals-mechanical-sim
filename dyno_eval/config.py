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
MOTOR_INERTIA = 0.00000371 # Final estimated value
FLYWHEEL_INERTIA = 0.0

# --- Analysis Parameters ---
SETTLING_THRESHOLD = 0.95 # Clip the analysis when velocity reaches 95% of its peak
STEADY_STATE_POINTS_TO_KEEP = 0 # Keep a few points of the plateau for context
VELOCITY_THRESHOLD = 10 # rev/s

# --- Filtering Parameters ---
# Parameters for the Savitzky-Golay filter used for calculating acceleration.
# Rule: SAVGOL_WINDOW_LENGTH must be an odd integer > SAVGOL_POLYORDER.
# For sparse data (low sample rate), use smaller values.
SAVGOL_WINDOW_LENGTH = 3
SAVGOL_POLYORDER = 2