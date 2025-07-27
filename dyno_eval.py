import pandas as pd
import matplotlib.pyplot as plt
import os

# Set the plot style globally for the entire script
plt.style.use('dark_background')

# --- Configuration ---
COLUMN_MAP = {
    "time": "time",
    "velocity": "ODrive 0 Velocity",
    "current": "ODrive 0 Current"
}
# ---------------------

# --- Preprocessing Functions ---

def sanitize_idle_current(df):
    """
    Finds and zeros out constant current values at the start and end of the log.
    """
    print("-> Running: Sanitize Idle Current")
    current_col = COLUMN_MAP["current"]

    # Process the start of the data
    initial_current = df.loc[0, current_col]
    series = df[current_col]
    
    if (series != initial_current).any():
        first_change_idx = (series != initial_current).idxmax()
        if first_change_idx > 0:
            print(f"   Sanitizing flat-line at start (value: {initial_current:.2f})")
            df.loc[0:first_change_idx-1, current_col] = 0
    else:
        if initial_current != 0:
             print(f"   Entire log is a flat-line (value: {initial_current:.2f}). Zeroing current.")
             df[current_col] = 0

    # Process the end of the data
    final_current = df.loc[df.index[-1], current_col]
    reversed_series = series.iloc[::-1]

    if (reversed_series != final_current).any():
        last_change_idx = (reversed_series != final_current).idxmax()
        if last_change_idx < df.index[-1] and last_change_idx > df.index[0]:
            print(f"   Sanitizing flat-line at end (value: {final_current:.2f})")
            df.loc[last_change_idx+1:, current_col] = 0
    
    return df

# --- Plotting Function ---

def create_motor_plot(df, title):
    """
    Creates a 2-subplot figure for velocity and current vs. time.
    Assumes it receives a clean, processed DataFrame.
    """
    time_col = COLUMN_MAP["time"]
    vel_col = COLUMN_MAP["velocity"]
    current_col = COLUMN_MAP["current"]

    fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f"Motor Performance: {title}", fontsize=16)

    # Velocity Subplot
    axes[0].plot(df[time_col], df[vel_col], color='cyan', label='Velocity')
    axes[0].set_ylabel('Velocity (rad/s)', fontsize=12)
    axes[0].grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.5)
    axes[0].legend(loc='upper left')

    # Current Subplot
    axes[1].plot(df[time_col], df[current_col], color='#FF00FF', label='Current')
    axes[1].set_ylabel('Iq Measured (A)', fontsize=12)
    axes[1].set_xlabel('Time (s)', fontsize=12)
    axes[1].grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.5)
    axes[1].legend(loc='upper left')

    fig.tight_layout(rect=[0, 0.03, 1, 0.96])


if __name__ == "__main__":
    data_subdir = 'ultra_mk2_data'
    cwd = os.getcwd()
    data_directory = os.path.join(cwd, data_subdir)

    # --- Select File ---
    # file_to_process = '1_torque_ramp_90A_limit.csv'
    file_to_process = 'no_flywheel.csv'

    full_filepath = os.path.join(data_directory, file_to_process)

    if not os.path.exists(full_filepath):
        print(f"Error: Target file not found at '{full_filepath}'")
    else:
        # 1. LOAD DATA
        print(f"Loading data from {file_to_process}...")
        motor_df = pd.read_csv(full_filepath)

        # 2. PREPROCESSING PIPELINE
        #    You can chain as many processing functions as you want here.
        motor_df = sanitize_idle_current(motor_df)
        # motor_df = some_other_filter(motor_df) # Example for the future
        # motor_df = and_another_one(motor_df)   # Example for the future
        
        # 3. PLOT
        print("Processing complete. Generating plot.")
        plot_title = file_to_process.replace('.csv', '')
        create_motor_plot(motor_df, plot_title)
        
        plt.show()

    print("Done.")