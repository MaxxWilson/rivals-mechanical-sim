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
# --- SIMULATION ENGINE & CONTROL LAWS ---
# =================================================================================

def step_sim(position, velocity, acceleration, dt):
    """Advances the physics simulation by one time step (dt)."""
    new_velocity = velocity + acceleration * dt
    new_position = position + new_velocity * dt
    return new_position, new_velocity

def control_law_drag_race(state):
    """Control law for constant max acceleration."""
    return state['max_accel']

def control_law_trapezoidal(state):
    """Control law for a symmetric accel/decel profile."""
    if state['position'] < state['distance_m'] / 2.0:
        return state['max_accel']
    else:
        return -state['max_accel']

def run_simulation(distance_m, max_accel, control_law_fn, dt=0.001):
    """
    Runs a generic simulation loop, using a callback to get acceleration.
    """
    time, position, velocity = 0, 0, 0
    t_series, pos_series, vel_series, accel_series = [0], [0], [0], [0]
    
    while position < distance_m:
        current_state = {
            'time': time, 'position': position, 'velocity': velocity,
            'distance_m': distance_m, 'max_accel': max_accel
        }
        current_accel = control_law_fn(current_state)
        position, velocity = step_sim(position, velocity, current_accel, dt)
        velocity = max(0, velocity)
        time += dt
        
        t_series.append(time)
        pos_series.append(position)
        vel_series.append(velocity)
        accel_series.append(current_accel)
        
    return {"time": np.array(t_series), "position": np.array(pos_series), "velocity": np.array(vel_series), "acceleration": np.array(accel_series)}

# =================================================================================
# --- OUTPUT FUNCTIONS ---
# =================================================================================

def print_summary(sim_data, title):
    """Prints a summary for a given simulation result."""
    print(f"--- {title} ---")
    header = f"{'Path':<12} | {'Peak Velocity (m/s)':<20} | {'Total Time (s)':<15}"
    print(header)
    print("-" * len(header))
    for name, data in sim_data.items():
        peak_vel = np.max(data['velocity'])
        total_time = data['time'][-1]
        print(f"{name:<12} | {peak_vel:<20.2f} | {total_time:<15.3f}")
    print("--------------------------------------------------")

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
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.2)

def plot_kinematics(sim_data, title):
    """Generates a 3x3 plot for pos, vel, and accel for a given simulation."""
    plt.style.use('dark_background')
    path_colors = {'Side': 'cyan', 'Diagonal': 'lime', 'Curved': 'magenta'}
    max_time = max(data['time'][-1] for data in sim_data.values())

    fig, axes = plt.subplots(3, 3, figsize=(15, 8), sharey='row', sharex=True, tight_layout=True)
    fig.suptitle(title, fontsize=16)

    for i, (name, data) in enumerate(sim_data.items()):
        color = path_colors[name]
        axes[0, i].plot(data['time'], data['position'], color=color)
        axes[0, i].set_title(f'{name} Path ({PATHS_IN[name]:.1f}")')
        axes[0, i].grid(True, linestyle='--', alpha=0.3)
        axes[1, i].plot(data['time'], data['velocity'], color=color)
        axes[1, i].grid(True, linestyle='--', alpha=0.3)
        axes[2, i].plot(data['time'], data['acceleration'], color=color)
        axes[2, i].set_xlabel('Time (s)')
        axes[2, i].grid(True, linestyle='--', alpha=0.3)

    axes[0, 0].set_ylabel('Position (m)')
    axes[1, 0].set_ylabel('Velocity (m/s)')
    axes[2, 0].set_ylabel('Acceleration (m/s^2)')
    axes[2, 0].set_xlim(0, max_time * 1.05)

# =================================================================================
# --- MAIN EXECUTION ---
# =================================================================================

def main():
    """Main function to run the simulation and outputs."""
    try:
        # --- Run Drag Race Sim ---
        drag_race_sim_data = {}
        for name, dist_m in PATHS_M.items():
            drag_race_sim_data[name] = run_simulation(dist_m, MAX_ACCELERATION_MS2, control_law_drag_race)
        
        # --- Run Trapezoidal Sim ---
        trapezoidal_sim_data = {}
        for name, dist_m in PATHS_M.items():
            trapezoidal_sim_data[name] = run_simulation(dist_m, MAX_ACCELERATION_MS2, control_law_trapezoidal)
        
        # --- Output Results ---
        print_summary(drag_race_sim_data, "Drag Race Simulation Results")
        print_summary(trapezoidal_sim_data, "Trapezoidal Profile Simulation Results")
        
        plot_field_paths()
        plot_kinematics(drag_race_sim_data, title='Kinematic Profiles (Drag Race)')
        plot_kinematics(trapezoidal_sim_data, title='Kinematic Profiles (Trapezoidal)')
        
        plt.show()

    except KeyboardInterrupt:
        print("\nSimulation aborted by user. Exiting.")
        sys.exit(0)

if __name__ == "__main__":
    main()