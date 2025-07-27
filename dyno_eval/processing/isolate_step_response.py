# processing/isolate_step_response.py
from config import COLUMN_MAP

def isolate_step_response(transient_df, settling_threshold, steady_state_points):
    vel_col = COLUMN_MAP["velocity"]
    peak_velocity = transient_df[vel_col].max()
    if (transient_df[vel_col] >= peak_velocity * settling_threshold).any():
        settling_point_idx = (transient_df[vel_col] >= peak_velocity * settling_threshold).idxmax()
    else:
        settling_point_idx = transient_df[vel_col].idxmax()
    end_slice_idx = settling_point_idx + steady_state_points
    last_valid_idx = transient_df.index[-1]
    final_idx = min(end_slice_idx, last_valid_idx)
    step_response_df = transient_df.loc[:final_idx].copy()
    return step_response_df