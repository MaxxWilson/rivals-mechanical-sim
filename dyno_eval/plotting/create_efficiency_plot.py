# plotting/create_efficiency_plot.py
import matplotlib.pyplot as plt
from config import COLUMN_MAP

def create_efficiency_plot(transients):
    """
    Creates a dedicated figure for pseudo-efficiency, showing the ramp-up
    and steady-state for each transient in a separate column.
    """
    print("-> Generating Pseudo-Efficiency Plot")
    if not transients:
        return

    num_cols = len(transients)
    fig, axes = plt.subplots(1, num_cols, figsize=(7 * num_cols, 5), sharex=True, sharey=True)
    fig.suptitle("Pseudo-Efficiency (Actual Torque / Ideal Torque)", fontsize=16)

    if num_cols == 1:
        axes = [axes] # Make it iterable for the loop
    
    plot_style = {'marker': '.', 'markersize': 4, 'linestyle': 'None', 'color': 'lightblue'}

    for i, transient_df in enumerate(transients):
        ax = axes[i]
        ax.plot(transient_df[COLUMN_MAP['time']], transient_df['pseudo_efficiency'], **plot_style)
        
        ax.set_title(f"Transient {i+1}")
        ax.set_xlabel("Time (s)")
        ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)
        
    axes[0].set_ylabel("Pseudo-Efficiency (%)")
    axes[0].set_ylim(0, 110) # Set a consistent y-axis from 0% to 110%
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])