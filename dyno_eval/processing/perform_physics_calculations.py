# processing/perform_physics_calculations.py
import numpy as np
from scipy.signal import savgol_filter
from config import COLUMN_MAP, MOTOR_KV, GEAR_RATIO, MOTOR_INERTIA, FLYWHEEL_INERTIA

def perform_physics_calculations(df):
    """
    Calculates all physics parameters IN THE MOTOR'S REFERENCE FRAME.
    The gear ratio is only used to reflect the flywheel inertia.
    """
    print("-> Running: Physics Calculations (at Motor Shaft)")

    kt = 60 / (2 * np.pi * MOTOR_KV)
    
    # --- CALCULATIONS AT THE MOTOR SHAFT ---
    
    # 1. Ideal Torque from motor current
    df['motor_torque_ideal'] = df[COLUMN_MAP['current']] * kt
    
    # 2. Motor Acceleration using a Savitzky-Golay filter for robustness
    # This is superior to diff() as it smooths the data while differentiating.
    vel_rad_s = df[COLUMN_MAP['velocity']] * (2 * np.pi)
    
    # Ensure we have a constant time step for the filter, use the median dt.
    dt = df[COLUMN_MAP['time']].diff().median()
    
    # The filter requires an odd window_length > polyorder.
    # These values need to be tuned based on data sampling rate.
    window_length = 15 # Must be odd
    polyorder = 3
    if len(vel_rad_s) > window_length:
        df['motor_acceleration_rad_s2'] = savgol_filter(
            vel_rad_s, 
            window_length=window_length, 
            polyorder=polyorder, 
            deriv=1, 
            delta=dt
        )
    else:
        # Fallback for very short dataframes, though filtering is preferred
        df['motor_acceleration_rad_s2'] = (vel_rad_s.diff() / df[COLUMN_MAP['time']].diff()).fillna(0)

    # 3. Total Inertia as seen by the motor
    flywheel_inertia_at_motor = FLYWHEEL_INERTIA / (GEAR_RATIO**2)
    total_inertia_at_motor = MOTOR_INERTIA + flywheel_inertia_at_motor
    
    # 4. Actual Torque required to accelerate the total inertia
    df['motor_torque_actual'] = df['motor_acceleration_rad_s2'] * total_inertia_at_motor
    
    # 5. Pseudo-Efficiency and Calculated Inertia
    with np.errstate(divide='ignore', invalid='ignore'):
        df['calculated_inertia'] = df['motor_torque_ideal'] / df['motor_acceleration_rad_s2']
        df['pseudo_efficiency'] = (df['motor_torque_actual'] / df['motor_torque_ideal']) * 100
    
    df.replace([np.inf, -np.inf], np.nan, inplace=True)
    
    print(f"   Total inertia at motor shaft: {total_inertia_at_motor:.8f} kg*m^2")
    return df