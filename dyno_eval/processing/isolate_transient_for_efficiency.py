# processing/isolate_transient_for_efficiency.py
from config import COLUMN_MAP

def isolate_transient_for_efficiency(transient_df):
    """
    Takes a full transient and clips it after the steady-state phase,
    removing the final "wind-down" period.
    """
    vel_col = COLUMN_MAP["velocity"]
    
    # Find the peak velocity and its location
    peak_velocity = transient_df[vel_col].max()
    peak_velocity_idx = transient_df[vel_col].idxmax()
    
    # Look at the data only after the peak has been reached
    df_after_peak = transient_df.loc[peak_velocity_idx:]
    
    # Find the first point where the velocity drops significantly (e.g., below 95% of peak)
    # This marks the beginning of the wind-down.
    wind_down_starts = df_after_peak[vel_col] < (peak_velocity * 0.95)
    
    if wind_down_starts.any():
        # If a wind-down is detected, find its starting index
        end_idx = wind_down_starts.idxmax()
        return transient_df.loc[:end_idx-1]
    else:
        # If no significant drop is found, the motor is at a plateau. Use the whole transient.
        return transient_df