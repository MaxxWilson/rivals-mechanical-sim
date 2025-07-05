# swerve_design_sim.py
import numpy as np
import matplotlib.pyplot as plt
import sys

def step_sim(position, velocity, acceleration, dt):
    """Advances the physics simulation by one time step (dt)."""
    new_velocity = velocity + acceleration * dt
    new_position = position + new_velocity * dt
    return new_position, new_velocity

def run_drag_race(distance_m, acceleration_ms2, dt=0.001):
    """
    Runs a simulation for a single path using constant acceleration
    and returns the full time-series data.
    """
    time, position, velocity = 0, 0, 0
    t_series, pos_series, vel_series = [0], [0], [0]
    while position < distance_m:
        position, velocity = step_sim(position, velocity, acceleration_ms2, dt)
        time += dt
        t_series.append(time)
        pos_series.append(position)
        vel_series.append(velocity)
    return {"time": np.array(t_series), "position": np.array(pos_series), "velocity": np.array(vel_series)}

def run_kinematic_simulation(params):
    """
    Performs all physics calculations and runs the simulation.
    """
    results = params.copy()
    inch_to_m = 0.0254

    # --- Calculations ---
    total_robot_width_in = params['ROBOT_DIM_IN'] + (2 * params['BUMPER_THICKNESS_IN'])
    path_side_in = params['FIELD_SIDE_IN'] - total_robot_width_in

    results['total_robot_width_in'] = total_robot_width_in
    results['paths_in'] = {
        'Side': path_side_in,
        'Diagonal': path_side_in * np.sqrt(2),
        'Curved': (np.pi / 2) * path_side_in
    }

    robot_mass_kg = params['ROBOT_WEIGHT_LBS'] * 0.453592
    max_tractive_force_n = robot_mass_kg * params['GRAVITY_MS2'] * params['COEFFICIENT_OF_FRICTION']
    max_acceleration_ms2 = max_tractive_force_n / robot_mass_kg
    results['max_acceleration_ms2'] = max_acceleration_ms2

    # --- Run Numerical Simulation for each path ---
    sim_data = {}
    for name, dist_in in results['paths_in'].items():
        sim_data[name] = run_drag_race(dist_in * inch_to_m, max_acceleration_ms2)
    results['sim_data'] = sim_data

    return results

def print_summary(results):
    """
    Prints the formatted results of the simulation.
    """
    # CORRECTED LINE: Use the 'results' dict directly, as it contains all params.
    params = results
    sim_data = results['sim_data']
    
    robot_mass_kg = params['ROBOT_WEIGHT_LBS'] * 0.453592
    max_tractive_force_n = robot_mass_kg * params['GRAVITY_MS2'] * params['COEFFICIENT_OF_FRICTION']
    wheel_radius_m = (params['WHEEL_DIAMETER_INCHES'] * 0.0254) / 2
    torque_per_motor_nm = (max_tractive_force_n * wheel_radius_m) / params['NUM_DRIVE_MOTORS']

    print("--- Swerve Design Simulation ---")
    print(f"Robot Mass: {robot_mass_kg:.2f} kg ({params['ROBOT_WEIGHT_LBS']} lbs)")
    print(f"Robot Footprint: {results['total_robot_width_in']:.2f}\" x {results['total_robot_width_in']:.2f}\"")
    print(f"Coefficient of Friction: {params['COEFFICIENT_OF_FRICTION']:.2f}")
    print("--------------------------------")
    print("--- Performance Limits ---")
    print(f"Max Tractive Force (Total): {max_tractive_force_n:.2f} N")
    print(f"Max Linear Acceleration: {results['max_acceleration_ms2']:.2f} m/s^2")
    print(f"Torque per Motor before Slip: {torque_per_motor_nm:.4f} Nm")
    print("--------------------------")
    print("--- Drag Race Simulation Results (Achievable Top Speed) ---")
    for name, data in sim_data.items():
        path_in = results['paths_in'][name]
        final_velocity = data['velocity'][-1]
        print(f"{name} Path ({path_in:.2f} in): \t\t{final_velocity:.2f} m/s")
    print("---------------------------------------------------------")

def plot_field_paths(results):
    """
    Generates a top-down plot of the field and simulated paths.
    """
    params = results
    paths_in = results['paths_in']
    total_robot_width_in = results['total_robot_width_in']
    
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')

    field_side = params['FIELD_SIDE_IN']
    robot_half_width = total_robot_width_in / 2

    # Field and Drivable Area
    ax.add_patch(plt.Rectangle((0, 0), field_side, field_side, facecolor='none', edgecolor='white', lw=2))
    ax.add_patch(plt.Rectangle((robot_half_width, robot_half_width), field_side - 2*robot_half_width, field_side - 2*robot_half_width, facecolor='none', edgecolor='gray', linestyle=':'))

    # Path Definitions
    start_pos = (robot_half_width, robot_half_width)
    end_pos_side = (field_side - robot_half_width, robot_half_width)
    end_pos_diag = (field_side - robot_half_width, field_side - robot_half_width)

    # Plotting Paths
    ax.plot([start_pos[0], end_pos_side[0]], [start_pos[1], end_pos_side[1]], color='cyan', label=f"Side ({paths_in['Side']:.1f}\")", linestyle='--')
    ax.plot([start_pos[0], end_pos_diag[0]], [start_pos[1], end_pos_diag[1]], color='lime', label=f"Diagonal ({paths_in['Diagonal']:.1f}\")", linestyle='--')
    
    arc_radius = paths_in['Side']
    arc_center = (robot_half_width, field_side - robot_half_width)
    theta = np.linspace(3 * np.pi / 2, 2 * np.pi, 100)
    ax.plot(arc_center[0] + arc_radius * np.cos(theta), arc_center[1] + arc_radius * np.sin(theta), color='magenta', label=f"Curved ({paths_in['Curved']:.1f}\")", linestyle='--')

    # Plot Options
    ax.set_xlim(0, field_side); ax.set_ylim(0, field_side)
    ax.set_xlabel('Field X-Position (inches)'); ax.set_ylabel('Field Y-Position (inches)')
    ax.set_title('Top-Down View of Simulated Paths'); ax.legend(loc='upper right'); ax.grid(True, alpha=0.2)

def plot_kinematics(results):
    """
    Generates a 2x3 plot for pos and vel for each path on a common time scale.
    """
    sim_data = results['sim_data']
    paths_in = results['paths_in']
    path_colors = {'Side': 'cyan', 'Diagonal': 'lime', 'Curved': 'magenta'}
    
    max_time = max(data['time'][-1] for data in sim_data.values())

    fig, axes = plt.subplots(2, 3, figsize=(15, 6), sharey='row', sharex=True, tight_layout=True)
    fig.suptitle('Kinematic Profiles (Drag Race)', fontsize=16)
    plt.style.use('dark_background')

    for i, (name, data) in enumerate(sim_data.items()):
        color = path_colors[name]
        axes[0, i].plot(data['time'], data['position'], color=color)
        axes[0, i].set_title(f'{name} Path ({paths_in[name]:.1f}")')
        axes[0, i].grid(True, linestyle='--', alpha=0.3)
        axes[1, i].plot(data['time'], data['velocity'], color=color)
        axes[1, i].set_xlabel('Time (s)')
        axes[1, i].grid(True, linestyle='--', alpha=0.3)

    axes[0, 0].set_ylabel('Position (m)')
    axes[1, 0].set_ylabel('Velocity (m/s)')
    axes[1, 0].set_xlim(0, max_time * 1.05)

def main():
    """
    Main function to define parameters and run the simulation.
    """
    try:
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
        plot_field_paths(results)
        plot_kinematics(results)

        plt.show() # Show all generated plots at the end

    except KeyboardInterrupt:
        print("\nSimulation aborted by user. Exiting.")
        sys.exit(0)

if __name__ == "__main__":
    main()