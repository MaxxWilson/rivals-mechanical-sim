# processing/sanitize_idle_current.py
from config import COLUMN_MAP

def sanitize_idle_current(df):
    print("-> Running: Sanitize Idle Current")
    current_col = COLUMN_MAP["current"]
    initial_current = df.loc[0, current_col]
    series = df[current_col]
    if (series != initial_current).any():
        first_change_idx = (series != initial_current).idxmax()
        if first_change_idx > 0:
            df.loc[0:first_change_idx-1, current_col] = 0
    return df