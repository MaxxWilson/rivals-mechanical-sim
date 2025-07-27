import pandas as pd
import numpy as np
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

# --- Physics & Hardware Constants ---
MOTOR_KV = 1450
GEAR_RATIO = 5.0
MOTOR_INERTIA = 0.000025
FLYWHEEL_INERTIA = 0.0015

# --- Analysis & Preprocessing Functions ---

def perform_physics_calculations(df):
    print("-> Running: Physics Calculations")
    kt = 60 / (2 * np.pi * MOTOR_KV)
    df['ideal_torque'] = df[COLUMN_MAP['current']] * kt * GEAR_RATIO
    time_diff = df[COLUMN_MAP['time']].diff()
    vel_diff = df[COLUMN_MAP['velocity']].diff()
    df['acceleration'] = vel_diff / time_diff
    df['smoothed_acceleration'] = df['acceleration'].rolling(window=5, center=True).mean()
    df.fillna(0, inplace=True)
    total_inertia = FLYWHEEL_INERTIA + (MOTOR_INERTIA * (GEAR_RATIO**2))
    df['actual_torque'] = df['smoothed_acceleration'] * total_inertia
    print(f"   Calculated Kt: {kt:.4f} Nm/A, Total Inertia: {total_inertia:.6f} kg*m^2")
    return df

def sanitize_idle_current(df):
    print("-> Running: Sanitize Idle Current")
    current_col = COLUMN_MAP["current"]
    # This logic is correct and remains unchanged.
    initial_current = df.loc[0, current_col]
    series = df[current_col]
    if (series != initial_current).any():
        first_change_idx = (series != initial_current).idxmax()
        if first_change_idx > 0:
            df.loc[0:first_change_idx-1, current_col] = 0
    else:
        if initial_current != 0:
             df[current_col] = 0
    final_current = df.loc[df.index[-1], current_col]
    reversed_series = series.iloc[::-1]
    if (reversed_series != final_current).any():
        last_change_idx = (reversed_series != final_current).idxmax()
        if last_change_idx < df.index[-1] and last_change_idx > df.index[0]:
            df.loc[last_change_idx+1:, current_col] = 0
    return df

def extract_transients(df, velocity_threshold=10):
    print(f"-> Running: Extract Transients (Threshold: {velocity_threshold} rad/s)")
    # This logic is correct and remains unchanged.
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
    print(f"   Found {len(transients)} transients.")
    return transients

def isolate_step_response(transient_df):
    # This logic is correct and remains unchanged.
    vel_col = COLUMN_MAP["velocity"]
    peak_velocity_idx = transient_df[vel_col].idxmax()
    step_response_df = transient_df.loc[:peak_velocity_idx].copy()
    return step_response_df

# --- Plotting Functions ---

def create_main_comparison_plot(df, step_responses, title):
    """
    MAIN PLOT: Full timeseries in Col 1, isolated STEP RESPONSES in subsequent columns.
    """
    print("-> Generating Main Comparison Plot (Step Responses)")
    num_cols = 1 + len(step_responses)
    fig, axes = plt.subplots(4, num_cols, figsize=(6 * num_cols, 16), sharex='col', sharey='row')
    fig.suptitle(f"Motor Step Response Analysis: {title}", fontsize=18)

    # Column 0: Full Timeseries
    axes[0, 0].set_title("Full Timeseries", fontsize=14)
    axes[0, 0].plot(df[COLUMN_MAP['time']], df[COLUMN_MAP['velocity']], color='cyan')
    axes[0, 0].set_ylabel('Velocity\n(rad/s)')
    axes[1, 0].plot(df[COLUMN_MAP['time']], df[COLUMN_MAP['current']], color='#FF00FF')
    axes[1, 0].set_ylabel('Current\n(A)')
    axes[2, 0].plot(df[COLUMN_MAP['time']], df['ideal_torque'], color='lime', label='Ideal')
    axes[2, 0].plot(df[COLUMN_MAP['time']], df['actual_torque'], color='yellow', linestyle='--', label='Actual')
    axes[2, 0].set_ylabel('Torque\n(Nm)')
    axes[2, 0].legend()
    axes[3, 0].plot(df[COLUMN_MAP['time']], df['smoothed_acceleration'], color='orange')
    axes[3, 0].set_ylabel('Acceleration\n(rad/s²)')
    axes[3, 0].set_xlabel('Time (s)')

    # Subsequent Columns: Individual Step Responses
    for i, step_df in enumerate(step_responses):
        col = i + 1
        axes[0, col].set_title(f"Step Response {i+1}", fontsize=14)
        axes[0, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['velocity']], color='cyan')
        axes[1, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['current']], color='#FF00FF')
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['ideal_torque'], color='lime')
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['actual_torque'], color='yellow', linestyle='--')
        axes[3, col].plot(step_df[COLUMN_MAP['time']], step_df['smoothed_acceleration'], color='orange')
        axes[3, col].set_xlabel('Time (s)')

    for ax_row in axes:
        for ax in ax_row:
            ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

def create_full_transient_comparison_plot(transients):
    """
    SECONDARY PLOT: Puts each full transient in a separate column.
    """
    print("-> Generating Full Transient Comparison Plot")
    if not transients:
        return # Don't create a plot if there's nothing to show

    num_cols = len(transients)
    fig, axes = plt.subplots(2, num_cols, figsize=(6 * num_cols, 8), sharex='col', sharey='row')
    fig.suptitle("Full Transient Comparison", fontsize=16)

    # Handle case where subplots returns a 1D array for a single column
    if num_cols == 1:
        axes = np.array([axes]).T

    for i, transient_df in enumerate(transients):
        ax_vel, ax_curr = axes[0, i], axes[1, i]
        ax_vel.set_title(f"Full Transient {i+1}", fontsize=14)
        ax_vel.plot(transient_df[COLUMN_MAP['time']], transient_df[COLUMN_MAP['velocity']], color='cyan')
        ax_curr.plot(transient_df[COLUMN_MAP['time']], transient_df[COLUMN_MAP['current']], color='#FF00FF')
        ax_curr.set_xlabel("Time (s)")

    # Set common labels
    axes[0, 0].set_ylabel("Velocity (rad/s)")
    axes[1, 0].set_ylabel("Current (A)")

    for ax_row in axes:
        for ax in ax_row:
            ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

if __name__ == "__main__":
    data_directory = os.path.join(os.getcwd(), 'ultra_mk2_data')
    file_to_process = 'no_flywheel.csv'
    full_filepath = os.path.join(data_directory, file_to_process)

    if not os.path.exists(full_filepath):
        print(f"Error: Target file not found at '{full_filepath}'")
    else:
        # 1. LOAD & NORMALIZE DATA
        print(f"Loading data from {file_to_process}...")
        motor_df = pd.read_csv(full_filepath)
        # Use absolute values for velocity and current to handle sign issues
        motor_df[COLUMN_MAP['velocity']] = motor_df[COLUMN_MAP['velocity']].abs()
        motor_df[COLUMN_MAP['current']] = motor_df[COLUMN_MAP['current']].abs()

        # 2. ANALYSIS PIPELINE
        motor_df = sanitize_idle_current(motor_df)
        motor_df = perform_physics_calculations(motor_df)
        transient_list = extract_transients(motor_df)
        
        # 3. PLOT & PROCESS
        if not transient_list:
            print("No transients found to plot.")
        else:
            step_responses = [isolate_step_response(t) for t in transient_list]
            print(f"-> Extracted {len(step_responses)} step-response periods.")

            # Generate the two figures
            create_main_comparison_plot(motor_df, step_responses, file_to_process.replace('.csv', ''))
            create_full_transient_comparison_plot(transient_list) # This line is updated
            
            plt.show()

    print("Done.")