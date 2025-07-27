# processing/isolate_step_response.py
from config import COLUMN_MAP

def isolate_step_response(transient_df, settling_threshold, steady_state_points):
    """
    Isolates the initial step response, cutting it off a few points after
    the velocity first reaches a settling_threshold of its peak value.
    """
    vel_col = COLUMN_MAP["velocity"]
    if transient_df.empty:
        return transient_df

    peak_velocity = transient_df[vel_col].max()
    
    # Find the first index where velocity crosses the settling threshold
    if (transient_df[vel_col] >= peak_velocity * settling_threshold).any():
        settling_point_idx = (transient_df[vel_col] >= peak_velocity * settling_threshold).idxmax()
    else:
        settling_point_idx = transient_df[vel_col].idxmax()

    # Determine the end point of the slice
    end_slice_idx = settling_point_idx + steady_state_points
    last_valid_idx = transient_df.index[-1]
    final_idx = min(end_slice_idx, last_valid_idx)

    return transient_df.loc[:final_idx].copy()