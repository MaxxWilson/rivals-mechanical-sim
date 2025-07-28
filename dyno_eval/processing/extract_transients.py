# processing/extract_transients.py
import pandas as pd
from config import COLUMN_MAP
from config import SETTLING_THRESHOLD, TRANSIENT_MIN_DURATION_S

def extract_transients(df, velocity_threshold):
    print(f"-> Running: Extract Transients (Threshold: {velocity_threshold} rev/s)")
    vel_col = COLUMN_MAP["velocity"]
    time_col = COLUMN_MAP["time"]
    
    last_velocity = 0.0

    starts = []
    ends = []

    is_active = False

    vel_col_pos = list(df.columns).index(vel_col) + 1
    time_col_pos = list(df.columns).index(time_col) + 1

    lookahead = 10

    for index in range(1, len(df) - lookahead-1):
        velocity = df.iloc[index][vel_col]
        last_velocity = df.iloc[index-1][vel_col]

        if not is_active and velocity > velocity_threshold and last_velocity < velocity_threshold:
            starts.append(index)
            is_active = True

        lookahead_avg = df.iloc[index+1 : index+lookahead+1][vel_col].mean()
        if is_active and velocity > SETTLING_THRESHOLD * lookahead_avg:
            ends.append(index)
            is_active = False

    if is_active and len(starts) > len(ends):
        ends.append(len(df) - 1)

    # Filter transients by minimum duration
    filtered_starts = []
    filtered_ends = []
    for start, end in zip(starts, ends):
        duration = df.loc[end, time_col] - df.loc[start, time_col]
        if duration >= TRANSIENT_MIN_DURATION_S:
            filtered_starts.append(start)
            filtered_ends.append(end)

    transients = []
    for start, end in zip(filtered_starts, filtered_ends):
        padded_start = max(df.index[0], start - 1)
        padded_end = min(df.index[-1], end + 1)
        transient_df = df.loc[padded_start:padded_end].copy()
        t_zero = df.loc[start, time_col]
        transient_df[time_col] -= t_zero
        transients.append(transient_df)
    return transients