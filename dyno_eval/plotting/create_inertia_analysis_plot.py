# plotting/create_inertia_analysis_plot.py
import matplotlib.pyplot as plt
from config import COLUMN_MAP

def create_inertia_analysis_plot(step_responses, final_inertia_est):
    """
    Creates a dedicated figure for inertia analysis, using new motor-centric columns.
    """
    print("-> Generating Inertia Analysis Plot")
    if not step_responses: return

    num_cols = len(step_responses)
    fig, axes = plt.subplots(4, num_cols, figsize=(7 * num_cols, 12), sharex='col')
    fig.suptitle("Inertia Calculation Analysis", fontsize=16)

    line_plot_style = {'marker': 'o', 'markersize': 2, 'linestyle': '-'}
    scatter_plot_style = {'color': 'white', 'marker': 'o', 'markersize': 2, 'linestyle': 'None'}

    for i, step_df in enumerate(step_responses):
        ax_vel, ax_accel, ax_torque, ax_inertia = (
            (axes[0, i], axes[1, i], axes[2, i], axes[3, i]) if num_cols > 1 
            else (axes[0], axes[1], axes[2], axes[3])
        )

        ax_vel.set_title(f"Transient {i+1}")
        ax_vel.plot(step_df[COLUMN_MAP['time']], step_df[COLUMN_MAP['velocity']], color='cyan', **line_plot_style)
        ax_accel.plot(step_df[COLUMN_MAP['time']], step_df['motor_acceleration_rad_s2'], color='orange', **line_plot_style)
        ax_torque.plot(step_df[COLUMN_MAP['time']], step_df['motor_torque_ideal'], color='lime', **line_plot_style)
        ax_inertia.plot(step_df[COLUMN_MAP['time']], step_df['calculated_inertia'], **scatter_plot_style)
        ax_inertia.set_xlabel("Time (s)")

    # Set common labels
    axes[0, 0].set_ylabel("Motor Velocity\n(rev/s)")
    axes[1, 0].set_ylabel("Motor Acceleration\n(rad/s²)")
    axes[2, 0].set_ylabel("Ideal Motor Torque\n(Nm)")
    axes[3, 0].set_ylabel("Est. Inertia\n(kg*m^2)")

    if final_inertia_est:
        y_lim_inertia = (0, final_inertia_est * 5)
        for i in range(num_cols):
            ax = axes[3, i] if num_cols > 1 else axes[3]
            ax.set_ylim(y_lim_inertia)

    for ax_row in axes:
        for ax in (ax_row if num_cols > 1 else [ax_row]):
             ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])