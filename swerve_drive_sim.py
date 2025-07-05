# swerve_design_sim.py
import numpy as np
import matplotlib.pyplot as plt

def run_kinematic_simulation(params):
    """
    Performs all physics calculations based on input parameters.
    """
    results = params.copy()
    inch_to_m = 0.0254

    # --- Calculations ---
    total_robot_width_in = params['ROBOT_DIM_IN'] + (2 * params['BUMPER_THICKNESS_IN'])
    path_side_in = params['FIELD_SIDE_IN'] - total_robot_width_in

    results['total_robot_width_in'] = total_robot_width_in
    results['path_side_in'] = path_side_in
    results['path_diagonal_in'] = path_side_in * np.sqrt(2)
    results['path_curve_in'] = (np.pi / 2) * path_side_in

    robot_mass_kg = params['ROBOT_WEIGHT_LBS'] * 0.453592
    max_tractive_force_n = robot_mass_kg * params['GRAVITY_MS2'] * params['COEFFICIENT_OF_FRICTION']
    max_acceleration_ms2 = max_tractive_force_n / robot_mass_kg
    results['max_acceleration_ms2'] = max_acceleration_ms2

    # --- Drag Race Velocities ---
    # v_f = sqrt(2 * a * d)
    results['v_final_side_ms'] = np.sqrt(2 * max_acceleration_ms2 * results['path_side_in'] * inch_to_m)
    results['v_final_diagonal_ms'] = np.sqrt(2 * max_acceleration_ms2 * results['path_diagonal_in'] * inch_to_m)
    results['v_final_curve_ms'] = np.sqrt(2 * max_acceleration_ms2 * results['path_curve_in'] * inch_to_m)

    return results

def print_summary(results):
    """
    Prints the formatted results of the simulation.
    """
    robot_mass_kg = results['ROBOT_WEIGHT_LBS'] * 0.453592
    max_tractive_force_n = robot_mass_kg * results['GRAVITY_MS2'] * results['COEFFICIENT_OF_FRICTION']
    wheel_radius_m = (results['WHEEL_DIAMETER_INCHES'] * 0.0254) / 2
    torque_per_motor_nm = (max_tractive_force_n * wheel_radius_m) / results['NUM_DRIVE_MOTORS']

    print("--- Swerve Design Simulation ---")
    print(f"Robot Mass: {robot_mass_kg:.2f} kg ({results['ROBOT_WEIGHT_LBS']} lbs)")
    print(f"Robot Footprint: {results['total_robot_width_in']:.2f}\" x {results['total_robot_width_in']:.2f}\"")
    print(f"Coefficient of Friction: {results['COEFFICIENT_OF_FRICTION']:.2f}")
    print("--------------------------------")
    print("--- Performance Limits ---")
    print(f"Max Tractive Force (Total): {max_tractive_force_n:.2f} N")
    print(f"Max Linear Acceleration: {results['max_acceleration_ms2']:.2f} m/s^2")
    print(f"Torque per Motor before Slip: {torque_per_motor_nm:.4f} Nm")
    print("--------------------------")
    print("--- Drag Race Simulation Results (Achievable Top Speed) ---")
    print(f"Side Path ({results['path_side_in']:.2f} in): \t\t{results['v_final_side_ms']:.2f} m/s")
    print(f"Diagonal Path ({results['path_diagonal_in']:.2f} in): {results['v_final_diagonal_ms']:.2f} m/s")
    print(f"Curved Path ({results['path_curve_in']:.2f} in): \t{results['v_final_curve_ms']:.2f} m/s")
    print("---------------------------------------------------------")

def plot_paths(results):
    """
    Generates a top-down plot of the field and simulated paths.
    """
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')

    field_side = results['FIELD_SIDE_IN']
    robot_half_width = results['total_robot_width_in'] / 2

    # Field and Drivable Area
    ax.add_patch(plt.Rectangle((0, 0), field_side, field_side, facecolor='none', edgecolor='white', lw=2))
    ax.add_patch(plt.Rectangle((robot_half_width, robot_half_width), field_side - 2*robot_half_width, field_side - 2*robot_half_width, facecolor='none', edgecolor='gray', linestyle=':'))

    # Path Definitions
    start_pos = (robot_half_width, robot_half_width)
    end_pos_side = (field_side - robot_half_width, robot_half_width)
    end_pos_diag = (field_side - robot_half_width, field_side - robot_half_width)

    # Plotting Paths
    ax.plot([start_pos[0], end_pos_side[0]], [start_pos[1], end_pos_side[1]], color='cyan', label=f"Side ({results['path_side_in']:.1f}\")", linestyle='--')
    ax.plot([start_pos[0], end_pos_diag[0]], [start_pos[1], end_pos_diag[1]], color='lime', label=f"Diagonal ({results['path_diagonal_in']:.1f}\")", linestyle='--')
    
    arc_radius = results['path_side_in']
    arc_center = (robot_half_width, field_side - robot_half_width)
    theta = np.linspace(3 * np.pi / 2, 2 * np.pi, 100)
    ax.plot(arc_center[0] + arc_radius * np.cos(theta), arc_center[1] + arc_radius * np.sin(theta), color='magenta', label=f"Curved ({results['path_curve_in']:.1f}\")", linestyle='--')

    # Plot Options
    ax.set_xlim(0, field_side)
    ax.set_ylim(0, field_side)
    ax.set_xlabel('Field X-Position (inches)')
    ax.set_ylabel('Field Y-Position (inches)')
    ax.set_title('Top-Down View of Simulated Paths')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.2)
    plt.show()

def main():
    """
    Main function to define parameters and run the simulation.
    """
    params = {
        'FIELD_SIDE_IN': 144.0,
        'ROBOT_DIM_IN': 12.0,
        'BUMPER_THICKNESS_IN': 1.0,
        'ROBOT_WEIGHT_LBS': 20.0,
        'NUM_DRIVE_MOTORS': 8,
        'WHEEL_DIAMETER_INCHES': 2.5,
        'COEFFICIENT_OF_FRICTION': 1.2,
        'GRAVITY_MS2': 9.81
    }

    results = run_kinematic_simulation(params)
    print_summary(results)
    plot_paths(results)

if __name__ == "__main__":
    main()