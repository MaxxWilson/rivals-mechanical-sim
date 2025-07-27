# plotting/create_main_comparison_plot.py
import matplotlib.pyplot as plt
from config import COLUMN_MAP

def create_main_comparison_plot(df, step_responses, title):
    """
    MAIN PLOT: Updated to show motor-centric values.
    """
    print("-> Generating Main Comparison Plot (4-row)")
    num_cols = 1 + len(step_responses)
    
    fig, axes = plt.subplots(4, num_cols, figsize=(6 * num_cols, 16), sharex='col', sharey='row')
    fig.suptitle(f"Motor Step Response Analysis: {title}", fontsize=18)

    # --- Column 0: Full Timeseries ---
    axes[0, 0].set_title("Full Timeseries", fontsize=14)
    axes[0, 0].plot(df[COLUMN_MAP['time']], df[COLUMN_MAP['velocity']], color='cyan')
    axes[1, 0].plot(df[COLUMN_MAP['time']], df[COLUMN_MAP['current']], color='#FF00FF')
    # Plotting new motor torque columns
    axes[2, 0].plot(df[COLUMN_MAP['time']], df['motor_torque_ideal'], color='lime', label='Ideal Motor Torque')
    axes[2, 0].plot(df[COLUMN_MAP['time']], df['motor_torque_actual'], color='yellow', linestyle='--', label='Actual Motor Torque')
    axes[3, 0].plot(df[COLUMN_MAP['time']], df['motor_acceleration_rad_s2'], color='orange')

    # --- Subsequent Columns: Individual Step Responses ---
    marker_style = {'marker': 'o', 'markersize': 2, 'linestyle': 'None', 'color': 'white'}
    
    for i, step_df in enumerate(step_responses):
        col = i + 1
        axes[0, col].set_title(f"Step Response {i+1}", fontsize=14)
        axes[0, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['velocity']], color='cyan')
        axes[0, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['velocity']], **marker_style)
        axes[1, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['current']], color='#FF00FF')
        axes[1, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['current']], **marker_style)
        # Plotting new motor torque columns
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['motor_torque_ideal'], color='lime')
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['motor_torque_ideal'], **marker_style)
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['motor_torque_actual'], color='yellow', linestyle='--')
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['motor_torque_actual'], **marker_style)
        axes[3, col].plot(step_df[COLUMN_MAP['time']], step_df['motor_acceleration_rad_s2'], color='orange')
        axes[3, col].plot(step_df[COLUMN_MAP['time']], step_df['motor_acceleration_rad_s2'], **marker_style)

    # Set labels and legends
    axes[0, 0].set_ylabel('Motor Velocity\n(rev/s)')
    axes[1, 0].set_ylabel('Motor Current\n(A)')
    axes[2, 0].set_ylabel('Motor Torque\n(Nm)')
    axes[3, 0].set_ylabel('Motor Acceleration\n(rad/s²)')
    axes[2, 0].legend()
    for i in range(num_cols):
        axes[3, i].set_xlabel('Time (s)')

    for ax_row in axes:
        for ax in ax_row:
            ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)
            
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])