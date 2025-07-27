# plotting/create_main_comparison_plot.py
import matplotlib.pyplot as plt
from config import COLUMN_MAP

def create_main_comparison_plot(df, step_responses, title):
    """
    MAIN PLOT: A clean 4-row system overview. The inertia row has been removed.
    """
    print("-> Generating Main Comparison Plot (4-row)")
    num_cols = 1 + len(step_responses)
    
    fig, axes = plt.subplots(4, num_cols, figsize=(6 * num_cols, 16), sharex='col', sharey='row')
    fig.suptitle(f"Motor Step Response Analysis: {title}", fontsize=18)

    # --- Column 0: Full Timeseries (Lines Only) ---
    axes[0, 0].set_title("Full Timeseries", fontsize=14)
    axes[0, 0].plot(df[COLUMN_MAP['time']], df[COLUMN_MAP['velocity']], color='cyan')
    axes[1, 0].plot(df[COLUMN_MAP['time']], df[COLUMN_MAP['current']], color='#FF00FF')
    axes[2, 0].plot(df[COLUMN_MAP['time']], df['ideal_torque'], color='lime', label='Ideal')
    axes[2, 0].plot(df[COLUMN_MAP['time']], df['actual_torque'], color='yellow', linestyle='--', label='Actual')
    axes[3, 0].plot(df[COLUMN_MAP['time']], df['acceleration_rad_s2'], color='orange')

    # --- Subsequent Columns: Individual Step Responses (Lines with White Markers) ---
    marker_style = {'marker': 'o', 'markersize': 2, 'linestyle': 'None', 'color': 'white'}
    
    for i, step_df in enumerate(step_responses):
        col = i + 1
        axes[0, col].set_title(f"Step Response {i+1}", fontsize=14)
        
        axes[0, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['velocity']], color='cyan')
        axes[0, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['velocity']], **marker_style)
        axes[1, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['current']], color='#FF00FF')
        axes[1, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['current']], **marker_style)
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['ideal_torque'], color='lime')
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['ideal_torque'], **marker_style)
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['actual_torque'], color='yellow', linestyle='--')
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['actual_torque'], **marker_style)
        axes[3, col].plot(step_df[COLUMN_MAP['time']], step_df['acceleration_rad_s2'], color='orange')
        axes[3, col].plot(step_df[COLUMN_MAP['time']], step_df['acceleration_rad_s2'], **marker_style)

    # Set labels and legends
    axes[0, 0].set_ylabel('Velocity\n(rev/s)')
    axes[1, 0].set_ylabel('Current\n(A)')
    axes[2, 0].set_ylabel('Torque\n(Nm)')
    axes[3, 0].set_ylabel('Acceleration\n(rad/s²)')
    axes[2, 0].legend()
    for i in range(num_cols):
        axes[3, i].set_xlabel('Time (s)')

    for ax_row in axes:
        for ax in ax_row:
            ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)
            
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])