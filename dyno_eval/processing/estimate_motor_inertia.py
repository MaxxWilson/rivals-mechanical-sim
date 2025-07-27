# processing/estimate_motor_inertia.py
import numpy as np
import pandas as pd
from sklearn.linear_model import TheilSenRegressor
from config import COLUMN_MAP, MOTOR_KV

def estimate_motor_inertia(step_responses):
    """
    Estimates motor inertia using a Theil-Sen Regressor, which is highly
    robust to outliers in sparse data.
    """
    print("-> Running: Estimate Motor Inertia (Theil-Sen Method)")
    
    kt = 60 / (2 * np.pi * MOTOR_KV)
    
    # 1. Pool data
    all_data_list = []
    for step_df in step_responses:
        all_data_list.append(pd.DataFrame({
            'acceleration': step_df['acceleration_rad_s2'],
            'torque': step_df[COLUMN_MAP['current']] * kt
        }))
    
    if not all_data_list: return None, None
    combined_df = pd.concat(all_data_list).dropna()
    
    X = combined_df['acceleration'].values.reshape(-1, 1)
    y = combined_df['torque'].values

    if len(X) < 2:
        print("   Not enough data points for Theil-Sen fit.")
        return None, None

    # 2. Fit Theil-Sen Regressor
    theil = TheilSenRegressor(random_state=42)
    theil.fit(X, y)
    
    inertia_estimate = theil.coef_[0]
    intercept_estimate = theil.intercept_
    
    # 3. Identify outliers based on the robust Theil-Sen fit
    residuals = y - theil.predict(X)
    is_outlier = np.abs(residuals) > 2 * np.std(residuals)
    
    plot_data = {
        'inliers': combined_df[~is_outlier],
        'outliers': combined_df[is_outlier],
        'slope': inertia_estimate,
        'intercept': intercept_estimate
    }
    
    print(f"   Theil-Sen found {np.sum(~is_outlier)} inliers and {np.sum(is_outlier)} outlier(s).")
    
    return inertia_estimate, plot_data