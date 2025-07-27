# plotting/create_efficiency_plot.py
import matplotlib.pyplot as plt
from config import COLUMN_MAP

def create_efficiency_plot(transients):
    """
    Creates a dedicated figure for pseudo-efficiency, based on motor torques.
    """
    print("-> Generating Filtered Pseudo-Efficiency Plot")
    if not transients:
        return

    num_cols = len(transients)
    fig, axes = plt.subplots(1, num_cols, figsize=(7 * num_cols, 5), sharex=True, sharey=True)
    fig.suptitle("Pseudo-Efficiency (Actual Motor Torque / Ideal Motor Torque)", fontsize=16)

    if num_cols == 1:
        axes = [axes]
    
    plot_style = {'marker': 'o', 'markersize': 2, 'linestyle': '-', 'color': 'lightblue'}

    for i, transient_df in enumerate(transients):
        ax = axes[i]
        
        peak_torque = transient_df['motor_torque_ideal'].max()
        filtered_df = transient_df[
            (transient_df['motor_acceleration_rad_s2'] > 0) &
            (transient_df['motor_torque_ideal'] > 0.1 * peak_torque)
        ]

        if not filtered_df.empty:
            ax.plot(filtered_df[COLUMN_MAP['time']], filtered_df['pseudo_efficiency'], **plot_style)
        
        ax.set_title(f"Transient {i+1}")
        ax.set_xlabel("Time (s)")
        ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)
        
    axes[0].set_ylabel("Pseudo-Efficiency (%)")
    axes[0].set_ylim(0, 110)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])