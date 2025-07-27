# processing/estimate_motor_inertia.py
import numpy as np
import pandas as pd
from sklearn.linear_model import TheilSenRegressor

def estimate_motor_inertia(step_responses):
    """
    Estimates motor inertia using a Theil-Sen Regressor on the pooled data
    from the clean step response windows.
    """
    print("-> Running: Estimate Motor Inertia (Theil-Sen on Clipped Data)")
    
    # 1. Pool data ONLY from the clean step_responses
    all_data_list = []
    for step_df in step_responses:
        # THE FIX IS HERE: Use the pre-calculated columns directly. No more recalculating torque.
        all_data_list.append(pd.DataFrame({
            'acceleration': step_df['motor_acceleration_rad_s2'],
            'torque': step_df['motor_torque_ideal'] 
        }))
    
    if not all_data_list: return None, None
    combined_df = pd.concat(all_data_list).dropna()
    
    # Remove points with zero or near-zero acceleration, as they offer no info
    # and can cause issues with the regression's stability.
    combined_df = combined_df[combined_df['acceleration'].abs() > 1e-1]

    X = combined_df['acceleration'].values.reshape(-1, 1)
    y = combined_df['torque'].values

    if len(X) < 2:
        return None, None

    # 2. Fit Theil-Sen Regressor on this clean, pooled data
    theil = TheilSenRegressor(random_state=42)
    theil.fit(X, y)
    
    inertia_estimate = theil.coef_[0]
    intercept_estimate = theil.intercept_
    
    # 3. Identify outliers based on the robust Theil-Sen fit
    residuals = y - theil.predict(X)
    is_outlier = np.abs(residuals) > 2.5 * np.std(residuals)
    
    plot_data = {
        'inliers': combined_df[~is_outlier],
        'outliers': combined_df[is_outlier],
        'slope': inertia_estimate,
        'intercept': intercept_estimate
    }
    
    print(f"   Theil-Sen using {np.sum(~is_outlier)} inliers and found {np.sum(is_outlier)} outlier(s).")
    
    return inertia_estimate, plot_data