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
    
    # 2. Motor Acceleration using exact dt (no averaging, no filtering)
    vel_rad_s = df[COLUMN_MAP['velocity']] * (2 * np.pi)
    time_diff = df[COLUMN_MAP['time']].diff() # Use exact dt for each row
    vel_diff = vel_rad_s.diff()
    df['motor_acceleration_rad_s2'] = (vel_diff / time_diff).fillna(0)
    
    # 3. Total Inertia as seen by the motor
    # Reflect the flywheel's inertia back to the motor shaft
    flywheel_inertia_at_motor = FLYWHEEL_INERTIA / (GEAR_RATIO**2)
    total_inertia_at_motor = MOTOR_INERTIA + flywheel_inertia_at_motor
    
    # 4. Actual Torque required to accelerate the total inertia
    df['motor_torque_actual'] = df['motor_acceleration_rad_s2'] * total_inertia_at_motor
    
    # 5. Pseudo-Efficiency (ratio of the two motor torques)
    with np.errstate(divide='ignore', invalid='ignore'):
        df['pseudo_efficiency'] = (df['motor_torque_actual'] / df['motor_torque_ideal']) * 100
    
    df.replace([np.inf, -np.inf], np.nan, inplace=True)
    
    print(f"   Total inertia at motor shaft: {total_inertia_at_motor:.8f} kg*m^2")
    return df