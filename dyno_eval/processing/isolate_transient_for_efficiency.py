# processing/isolate_transient_for_efficiency.py
from config import COLUMN_MAP

def isolate_transient_for_efficiency(transient_df):
    vel_col = COLUMN_MAP["velocity"]
    peak_velocity = transient_df[vel_col].max()
    peak_velocity_idx = transient_df[vel_col].idxmax()
    df_after_peak = transient_df.loc[peak_velocity_idx:]
    wind_down_starts = df_after_peak[vel_col] < (peak_velocity * 0.95)
    if wind_down_starts.any():
        end_idx = wind_down_starts.idxmax()
        return transient_df.loc[:end_idx-1]
    else:
        return transient_df