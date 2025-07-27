# plotting/create_main_comparison_plot.py
import matplotlib.pyplot as plt
from config import COLUMN_MAP

def create_main_comparison_plot(df, step_responses, title, final_inertia_est):
    print("-> Generating Main Comparison Plot (5-row)")
    num_cols = 1 + len(step_responses)
    
    # THE FIX IS HERE: sharey='row' handles axis sharing automatically and correctly.
    fig, axes = plt.subplots(5, num_cols, figsize=(6 * num_cols, 20), sharex='col', sharey='row')
    fig.suptitle(f"Motor Step Response Analysis: {title}", fontsize=18)

    # Column 0: Full Timeseries
    axes[0, 0].set_title("Full Timeseries", fontsize=14)
    axes[0, 0].plot(df[COLUMN_MAP['time']], df[COLUMN_MAP['velocity']], color='cyan')
    axes[1, 0].plot(df[COLUMN_MAP['time']], df[COLUMN_MAP['current']], color='#FF00FF')
    axes[2, 0].plot(df[COLUMN_MAP['time']], df['ideal_torque'], color='lime', label='Ideal')
    axes[2, 0].plot(df[COLUMN_MAP['time']], df['actual_torque'], color='yellow', linestyle='--', label='Actual')
    axes[3, 0].plot(df[COLUMN_MAP['time']], df['acceleration_rad_s2'], color='orange')
    axes[4, 0].plot(df[COLUMN_MAP['time']], df['calculated_inertia'], color='white', marker='.', linestyle='None')

    # Subsequent Columns: Individual Step Responses
    for i, step_df in enumerate(step_responses):
        col = i + 1
        axes[0, col].set_title(f"Step Response {i+1}", fontsize=14)
        axes[0, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['velocity']], color='cyan')
        axes[1, col].plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['current']], color='#FF00FF')
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['ideal_torque'], color='lime')
        axes[2, col].plot(step_df[COLUMN_MAP['time']], step_df['actual_torque'], color='yellow', linestyle='--')
        axes[3, col].plot(step_df[COLUMN_MAP['time']], step_df['acceleration_rad_s2'], color='orange')
        axes[4, col].plot(step_df[COLUMN_MAP['time']], step_df['calculated_inertia'], color='white', marker='.', linestyle='None')

    # Set labels and limits
    axes[0, 0].set_ylabel('Velocity\n(rev/s)')
    axes[1, 0].set_ylabel('Current\n(A)')
    axes[2, 0].set_ylabel('Torque\n(Nm)')
    axes[3, 0].set_ylabel('Acceleration\n(rad/s²)')
    axes[4, 0].set_ylabel('Inst. Inertia\n(kg*m^2)')
    axes[2, 0].legend()
    for i in range(num_cols):
        axes[4, i].set_xlabel('Time (s)')
    if final_inertia_est:
        y_lim_inertia = (0, final_inertia_est * 5)
        # Apply the y-limit to all inertia subplots
        for i in range(num_cols):
            axes[4, i].set_ylim(y_lim_inertia)
            
    for ax_row in axes:
        for ax in ax_row:
            ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)
            
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])