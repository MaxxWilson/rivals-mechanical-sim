# processing/extract_transients.py
import pandas as pd
from config import COLUMN_MAP

def extract_transients(df, velocity_threshold):
    print(f"-> Running: Extract Transients (Threshold: {velocity_threshold} rev/s)")
    vel_col = COLUMN_MAP["velocity"]
    time_col = COLUMN_MAP["time"]
    is_active = df[vel_col] > velocity_threshold
    starts = df[is_active & ~is_active.shift(1).fillna(False)].index
    ends = df[~is_active & is_active.shift(1).fillna(False)].index
    if len(starts) > len(ends):
        ends = ends.append(pd.Index([df.index[-1]]))
    transients = []
    for start, end in zip(starts, ends):
        padded_start = max(df.index[0], start - 1)
        padded_end = min(df.index[-1], end + 1)
        transient_df = df.loc[padded_start:padded_end].copy()
        t_zero = df.loc[start, time_col]
        transient_df[time_col] -= t_zero
        transients.append(transient_df)
    return transients