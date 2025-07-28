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
MOTOR_INERTIA = 0.00000371 # Final estimated value
FLYWHEEL_INERTIA = (2551.676 + 0.235)/1000.0/1000.0

# --- Analysis Parameters ---
SETTLING_THRESHOLD = 0.98 # Clip the analysis when velocity reaches 95% of its peak
STEADY_STATE_POINTS_TO_KEEP = 0 # Keep a few points of the plateau for context
VELOCITY_THRESHOLD = 10 # rev/s
TRANSIENT_MIN_DURATION_S = 0.03

# --- Filtering Parameters ---
# Parameters for the Savitzky-Golay filter used for calculating acceleration.
# Rule: SAVGOL_WINDOW_LENGTH must be an odd integer > SAVGOL_POLYORDER.
# For sparse data (low sample rate), use smaller values.
SAVGOL_WINDOW_LENGTH = 3
SAVGOL_POLYORDER = 2