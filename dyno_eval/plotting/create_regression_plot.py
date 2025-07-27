# plotting/create_regression_plot.py
import matplotlib.pyplot as plt
import numpy as np

def create_regression_plot(plot_data):
    """
    Visualizes the Theil-Sen regression on the clean analysis window,
    highlighting inliers and outliers.
    """
    print("-> Generating Theil-Sen Regression Plot")
    if not plot_data:
        return

    fig, ax = plt.subplots(1, 1, figsize=(10, 7))
    fig.suptitle("Theil-Sen Torque vs. Acceleration Fit (Analysis Window)", fontsize=16)

    # Plot the inlier data points
    ax.scatter(plot_data['inliers']['acceleration'], plot_data['inliers']['torque'], 
               alpha=0.7, label='Inlier Data Points')
    
    # Plot the rejected outlier points
    ax.scatter(plot_data['outliers']['acceleration'], plot_data['outliers']['torque'], 
               edgecolor='red', facecolor='none', s=100, label='Rejected Outlier(s)')
    
    # Plot the line of best fit from the Theil-Sen estimator
    # Use the actual data range for a tight fit line
    x_min = plot_data['inliers']['acceleration'].min()
    x_max = plot_data['inliers']['acceleration'].max()
    x_fit_range = np.linspace(x_min, x_max, 2)
    y_fit = plot_data['slope'] * x_fit_range + plot_data['intercept']
    
    fit_label = f"Theil-Sen Fit (Inertia={plot_data['slope']:.8f} kg*m²)"
    ax.plot(x_fit_range, y_fit, color='red', linestyle='--', label=fit_label)
    
    ax.set_xlabel("Acceleration (rad/s²)")
    ax.set_ylabel("Motor Torque (Nm)")
    ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)
    ax.legend()
    
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])