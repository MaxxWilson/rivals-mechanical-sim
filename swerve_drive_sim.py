# swerve_design_sim.py
import numpy as np
import matplotlib.pyplot as plt
import sys

# =================================================================================
# --- GLOBAL CONSTANTS AND ONE-TIME CALCULATIONS ---
# =================================================================================

# --- Base Dimensions (in Inches) ---
FIELD_SIDE_IN = 144.0
ROBOT_DIM_IN = 12.0
BUMPER_THICKNESS_IN = 1.0

# --- Robot Constants ---
ROBOT_WEIGHT_LBS = 20.0
NUM_DRIVE_MOTORS = 8
WHEEL_DIAMETER_INCHES = 2.5

# --- Environment Constants ---
COEFFICIENT_OF_FRICTION = 1.2
GRAVITY_MS2 = 9.81
INCHES_TO_METERS = 0.0254

# --- Derived Constants (Calculated Once) ---
ROBOT_MASS_KG = ROBOT_WEIGHT_LBS * 0.453592
TOTAL_ROBOT_WIDTH_IN = ROBOT_DIM_IN + (2 * BUMPER_THICKNESS_IN)
WHEEL_RADIUS_M = (WHEEL_DIAMETER_INCHES * INCHES_TO_METERS) / 2

# Path lengths in inches
PATH_SIDE_IN = FIELD_SIDE_IN - TOTAL_ROBOT_WIDTH_IN
PATH_DIAGONAL_IN = (FIELD_SIDE_IN - TOTAL_ROBOT_WIDTH_IN) * np.sqrt(2)
ARC_RADIUS_IN = PATH_SIDE_IN
PATH_CURVE_IN = (np.pi / 2) * ARC_RADIUS_IN
PATHS_IN = {'Side': PATH_SIDE_IN, 'Diagonal': PATH_DIAGONAL_IN, 'Curved': PATH_CURVE_IN}

# Path lengths in meters
PATHS_M = {name: dist_in * INCHES_TO_METERS for name, dist_in in PATHS_IN.items()}

# Performance Limits
MAX_TRACTIVE_FORCE_N = ROBOT_MASS_KG * GRAVITY_MS2 * COEFFICIENT_OF_FRICTION
MAX_ACCELERATION_MS2 = MAX_TRACTIVE_FORCE_N / ROBOT_MASS_KG
TORQUE_PER_MOTOR_BEFORE_SLIP_NM = (MAX_TRACTIVE_FORCE_N * WHEEL_RADIUS_M) / NUM_DRIVE_MOTORS

# =================================================================================
# --- SIMULATION ENGINE ---
# =================================================================================

def step_sim(position, velocity, acceleration, dt):
    """Advances the physics simulation by one time step (dt)."""
    new_velocity = velocity + acceleration * dt
    new_position = position + new_velocity * dt
    return new_position, new_velocity

def run_drag_race(distance_m, acceleration_ms2, dt=0.001):
    """Runs a constant acceleration simulation and returns the time-series data."""
    time, position, velocity = 0, 0, 0
    t_series, pos_series, vel_series = [0], [0], [0]
    while position < distance_m:
        position, velocity = step_sim(position, velocity, acceleration_ms2, dt)
        time += dt
        t_series.append(time)
        pos_series.append(position)
        vel_series.append(velocity)
    return {"time": np.array(t_series), "position": np.array(pos_series), "velocity": np.array(vel_series)}

# =================================================================================
# --- OUTPUT FUNCTIONS ---
# =================================================================================

def print_summary(sim_data):
    """Prints the formatted results of the simulation."""
    print("--- Swerve Design Simulation ---")
    print(f"Robot Mass: {ROBOT_MASS_KG:.2f} kg ({ROBOT_WEIGHT_LBS} lbs)")
    print(f"Robot Footprint: {TOTAL_ROBOT_WIDTH_IN:.2f}\" x {TOTAL_ROBOT_WIDTH_IN:.2f}\"")
    print(f"Coefficient of Friction: {COEFFICIENT_OF_FRICTION:.2f}")
    print("--------------------------------")
    print("--- Performance Limits ---")
    print(f"Max Tractive Force (Total): {MAX_TRACTIVE_FORCE_N:.2f} N")
    print(f"Max Linear Acceleration: {MAX_ACCELERATION_MS2:.2f} m/s^2")
    print(f"Torque per Motor before Slip: {TORQUE_PER_MOTOR_BEFORE_SLIP_NM:.4f} Nm")
    print("--------------------------")
    print("--- Drag Race Simulation Results (Achievable Top Speed) ---")
    for name, data in sim_data.items():
        final_velocity = data['velocity'][-1]
        print(f"{name} Path ({PATHS_IN[name]:.2f} in): \t\t{final_velocity:.2f} m/s")
    print("---------------------------------------------------------")

def plot_field_paths():
    """Generates a top-down plot of the field and simulated paths."""
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')

    robot_half_width = TOTAL_ROBOT_WIDTH_IN / 2

    # Field
    ax.add_patch(plt.Rectangle((0, 0), FIELD_SIDE_IN, FIELD_SIDE_IN, facecolor='none', edgecolor='white', lw=2))

    # Path Definitions
    start_pos = (robot_half_width, robot_half_width)
    end_pos_side = (FIELD_SIDE_IN - robot_half_width, robot_half_width)
    end_pos_diag = (FIELD_SIDE_IN - robot_half_width, FIELD_SIDE_IN - robot_half_width)

    # Plotting Paths
    ax.plot([start_pos[0], end_pos_side[0]], [start_pos[1], end_pos_side[1]], color='cyan', label=f"Side ({PATHS_IN['Side']:.1f}\")", linestyle='--')
    ax.plot([start_pos[0], end_pos_diag[0]], [start_pos[1], end_pos_diag[1]], color='lime', label=f"Diagonal ({PATHS_IN['Diagonal']:.1f}\")", linestyle='--')
    
    arc_center = (robot_half_width, FIELD_SIDE_IN - robot_half_width)
    theta = np.linspace(3 * np.pi / 2, 2 * np.pi, 100)
    ax.plot(arc_center[0] + ARC_RADIUS_IN * np.cos(theta), arc_center[1] + ARC_RADIUS_IN * np.sin(theta), color='magenta', label=f"Curved ({PATHS_IN['Curved']:.1f}\")", linestyle='--')

    # Plot Options
    ax.set_xlim(0, FIELD_SIDE_IN)
    ax.set_ylim(0, FIELD_SIDE_IN)
    ax.set_xlabel('Field X-Position (inches)')
    ax.set_ylabel('Field Y-Position (inches)')
    ax.set_title('Top-Down View of Simulated Paths')
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.2)

def plot_kinematics(sim_data):
    """Generates a 2x3 plot for pos and vel for each path."""
    plt.style.use('dark_background')
    path_colors = {'Side': 'cyan', 'Diagonal': 'lime', 'Curved': 'magenta'}
    max_time = max(data['time'][-1] for data in sim_data.values())

    fig, axes = plt.subplots(2, 3, figsize=(15, 6), sharey='row', sharex=True, tight_layout=True)
    fig.suptitle('Kinematic Profiles (Drag Race)', fontsize=16)

    for i, (name, data) in enumerate(sim_data.items()):
        color = path_colors[name]
        axes[0, i].plot(data['time'], data['position'], color=color)
        axes[0, i].set_title(f'{name} Path ({PATHS_IN[name]:.1f}")')
        axes[0, i].grid(True, linestyle='--', alpha=0.3)
        axes[1, i].plot(data['time'], data['velocity'], color=color)
        axes[1, i].set_xlabel('Time (s)')
        axes[1, i].grid(True, linestyle='--', alpha=0.3)

    axes[0, 0].set_ylabel('Position (m)')
    axes[1, 0].set_ylabel('Velocity (m/s)')
    axes[1, 0].set_xlim(0, max_time * 1.05)

# =================================================================================
# --- MAIN EXECUTION ---
# =================================================================================

def main():
    """Main function to run the simulation and outputs."""
    try:
        # Run the simulation for all paths
        sim_data = {}
        for name, dist_m in PATHS_M.items():
            sim_data[name] = run_drag_race(dist_m, MAX_ACCELERATION_MS2)
        
        # Output the results
        print_summary(sim_data)
        plot_field_paths()
        plot_kinematics(sim_data)
        plt.show()

    except KeyboardInterrupt:
        print("\nSimulation aborted by user. Exiting.")
        sys.exit(0)

if __name__ == "__main__":
    main()