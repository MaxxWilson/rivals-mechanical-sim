# processing/perform_physics_calculations.py
import numpy as np
from config import COLUMN_MAP, MOTOR_KV, GEAR_RATIO, MOTOR_INERTIA, FLYWHEEL_INERTIA

def perform_physics_calculations(df):
    print("-> Running: Physics Calculations")
    kt = 60 / (2 * np.pi * MOTOR_KV)
    df['ideal_torque'] = df[COLUMN_MAP['current']] * kt * GEAR_RATIO
    time_diff = df[COLUMN_MAP['time']].diff()
    vel_rad_s = df[COLUMN_MAP['velocity']] * (2 * np.pi)
    vel_diff_rad_s = vel_rad_s.diff()
    df['acceleration_rad_s2'] = (vel_diff_rad_s / time_diff).rolling(window=3).mean().fillna(0)
    total_inertia = FLYWHEEL_INERTIA + (MOTOR_INERTIA * (GEAR_RATIO**2))
    df['actual_torque'] = df['acceleration_rad_s2'] * total_inertia
    motor_torque = df[COLUMN_MAP['current']] * kt
    with np.errstate(divide='ignore', invalid='ignore'):
        df['calculated_inertia'] = motor_torque / df['acceleration_rad_s2']
    df.replace([np.inf, -np.inf], np.nan, inplace=True)
    print(f"   Calculated Kt: {kt:.4f} Nm/A, System Inertia: {total_inertia:.6f} kg*m^2")
    return df