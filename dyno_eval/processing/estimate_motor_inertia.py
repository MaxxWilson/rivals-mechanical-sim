# processing/estimate_motor_inertia.py
import numpy as np

def estimate_motor_inertia(step_responses):
    print("-> Running: Estimate Motor Inertia")
    all_inertia_estimates = []
    for i, step_df in enumerate(step_responses):
        peak_accel = step_df['acceleration_rad_s2'].max()
        clean_data_filter = (
            (step_df['acceleration_rad_s2'] > 0.10 * peak_accel) & 
            (step_df['acceleration_rad_s2'] < 0.95 * peak_accel)
        )
        clean_data = step_df[clean_data_filter]
        if not clean_data.empty:
            avg_inertia_for_transient = clean_data['calculated_inertia'].mean()
            all_inertia_estimates.append(avg_inertia_for_transient)
            print(f"   Transient {i+1}: Using {len(clean_data)} points, Estimated Inertia = {avg_inertia_for_transient:.8f} kg*m^2")
    if not all_inertia_estimates:
        return None
    final_estimate = np.mean(all_inertia_estimates)
    return final_estimate