# processing/perform_physics_calculations.py
import numpy as np
from scipy.signal import savgol_filter
from config import (COLUMN_MAP, MOTOR_KV, GEAR_RATIO, MOTOR_INERTIA, FLYWHEEL_INERTIA,
                    SAVGOL_WINDOW_LENGTH, SAVGOL_POLYORDER)

def perform_physics_calculations(df):
    """
    Calculates all physics parameters IN THE MOTOR'S REFERENCE FRAME.
    This now uses tunable Savitzky-Golay filter parameters from config.py.
    """
    print("-> Running: Physics Calculations (at Motor Shaft)")

    kt = 60 / (2 * np.pi * MOTOR_KV)
    
    # 1. Ideal Torque from motor current
    df['motor_torque_ideal'] = df[COLUMN_MAP['current']] * kt
    
    # 2. Motor Acceleration Calculations
    vel_rad_s = df[COLUMN_MAP['velocity']] * (2 * np.pi)
    time_s = df[COLUMN_MAP['time']]

    # 2a. Raw acceleration for plotting
    df['motor_acceleration_raw_rad_s2'] = (vel_rad_s.diff() / time_s.diff()).fillna(0)

    # 2b. Filtered acceleration using tunable parameters
    window = SAVGOL_WINDOW_LENGTH
    order = SAVGOL_POLYORDER
    
    # Robustness check for filter parameters
    if window <= order or window % 2 == 0:
        print(f"   ! Warning: Invalid Sav-Gol params (len={window}, order={order}). Check config.")
        window = max(order + 1, 3)
        if window % 2 == 0: window += 1
        print(f"   ! Using fallback params: (len={window}, order={order})")

    if len(vel_rad_s) > window:
        dt = time_s.diff().median()
        df['motor_acceleration_rad_s2'] = savgol_filter(
            vel_rad_s, 
            window_length=window, 
            polyorder=order, 
            deriv=1, 
            delta=dt
        )
    else:
        print("   ! Warning: Data too short for Sav-Gol filter. Using raw acceleration.")
        df['motor_acceleration_rad_s2'] = df['motor_acceleration_raw_rad_s2']

    # 3. Total Inertia (for 'actual' torque calculation)
    flywheel_inertia_at_motor = FLYWHEEL_INERTIA / (GEAR_RATIO**2)
    total_inertia_at_motor = MOTOR_INERTIA + flywheel_inertia_at_motor
    
    # 4. Actual Torque (uses the clean, filtered acceleration)
    df['motor_torque_actual'] = df['motor_acceleration_rad_s2'] * total_inertia_at_motor
    
    # 5. Pseudo-Efficiency and Calculated Inertia (uses clean acceleration)
    with np.errstate(divide='ignore', invalid='ignore'):
        df['calculated_inertia'] = df['motor_torque_ideal'] / df['motor_acceleration_rad_s2']
        df['pseudo_efficiency'] = (df['motor_torque_actual'] / df['motor_torque_ideal']) * 100
    
    df.replace([np.inf, -np.inf], np.nan, inplace=True)
    
    print(f"   Total inertia at motor shaft: {total_inertia_at_motor:.8f} kg*m^2")
    return df