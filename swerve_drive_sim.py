# swerve_design_sim.py
import numpy as np
import matplotlib.pyplot as plt
import sys

# =================================================================================
# --- GLOBAL CONSTANTS AND ONE-TIME CALCULATIONS ---
# =================================================================================
# Script Control
ENABLE_PLOTTING=False

# Unit Conversions
LBS_TO_KG = 0.453592
INCHES_TO_METERS = 0.0254

# Environmental Constants
COEFFICIENT_OF_FRICTION = 1.25
GRAVITY_MS2 = 9.81
FIELD_SIDE_IN = 144.0

# Robot Constants
ROBOT_DIM_IN = 12.0
BUMPER_THICKNESS_IN = 1.0
TOTAL_ROBOT_WIDTH_IN = ROBOT_DIM_IN + (2 * BUMPER_THICKNESS_IN)
ROBOT_WEIGHT_LBS = 22.5
ROBOT_MASS_KG = ROBOT_WEIGHT_LBS * 0.453592

# Path Lengths
PATH_SIDE_IN = FIELD_SIDE_IN - TOTAL_ROBOT_WIDTH_IN
PATH_DIAGONAL_IN = PATH_SIDE_IN * np.sqrt(2)
ARC_RADIUS_IN = PATH_SIDE_IN
PATH_CURVE_IN = (np.pi / 2) * ARC_RADIUS_IN
PATHS_IN = {'Side': PATH_SIDE_IN, 'Diagonal': PATH_DIAGONAL_IN, 'Curved': PATH_CURVE_IN}
PATHS_M = {name: dist_in * INCHES_TO_METERS for name, dist_in in PATHS_IN.items()}

# Power Constants
BATTERY_VOLTAGE_4S = 14.8
BATTERY_VOLTAGE_6S = 22.2
NUM_DRIVE_MOTORS = 8
SYSTEM_EFFICIENCY = 0.75 # Estimated efficiency from battery to wheel (85%)

# Drivetrain Performance
MAX_TRACTIVE_FORCE_N = ROBOT_MASS_KG * GRAVITY_MS2 * COEFFICIENT_OF_FRICTION
MAX_ACCELERATION_MS2 = MAX_TRACTIVE_FORCE_N / ROBOT_MASS_KG

# --- Robot Design Variables ---
WHEEL_RADIUS_M = (2.75 * INCHES_TO_METERS) / 2
DRIVE_KINEMATIC_CONSTANT_K = (13/44) * (30/14) / 2
STEER_KINEMATIC_CONSTANT_J = (13/44) / 2
TOTAL_DRIVE_RATIO = 1 / (2 * DRIVE_KINEMATIC_CONSTANT_K)

TARGET_V_MAX_MS = 3.0

# Drivetrain Performance Limits

TORQUE_PER_MOTOR_BEFORE_SLIP_NM = (MAX_TRACTIVE_FORCE_N * WHEEL_RADIUS_M) / (NUM_DRIVE_MOTORS * TOTAL_DRIVE_RATIO)

# Power System Requirements
P_MECH_PEAK_W = MAX_TRACTIVE_FORCE_N * TARGET_V_MAX_MS
P_ELEC_PEAK_W = P_MECH_PEAK_W / SYSTEM_EFFICIENCY
I_TOTAL_REQUIRED_A_4S = P_ELEC_PEAK_W / BATTERY_VOLTAGE_4S
I_TOTAL_REQUIRED_A_6S = P_ELEC_PEAK_W / BATTERY_VOLTAGE_6S
I_PER_MOTOR_REQUIRED_A_4S = I_TOTAL_REQUIRED_A_4S / NUM_DRIVE_MOTORS
I_PER_MOTOR_REQUIRED_A_6S = I_TOTAL_REQUIRED_A_6S / NUM_DRIVE_MOTORS

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

def control_law_trapezoidal_unlimited(state):
    """Control law for a symmetric accel/decel profile without a speed limit."""
    if state['position'] < state['distance_m'] / 2.0:
        return state['max_accel']
    else:
        return -state['max_accel']

def control_law_trapezoidal_limited(state):
    """Control law for a trapezoidal profile with a max velocity limit."""
    v_peak_triangular = np.sqrt(state['distance_m'] * state['max_accel'])
    actual_v_max = min(state['target_v_max'], v_peak_triangular)
    braking_distance = (actual_v_max**2) / (2 * state['max_accel'])

    if state['position'] >= (state['distance_m'] - braking_distance):
        return -state['max_accel']
    elif state['velocity'] >= actual_v_max:
        return 0
    else:
        return state['max_accel']

def run_simulation(distance_m, max_accel, control_law_fn, target_v_max=None, dt=0.00001, max_sim_time_s = 15.0):
    """Runs a generic simulation loop, using a callback to get acceleration."""
    time, position, velocity = 0, 0, 0
    t_series, pos_series, vel_series, accel_series = [0], [0], [0], [0]

    while position < distance_m:
        if time >= max_sim_time_s:
            print(f"\n--- SIM WARNING: Max time of {max_sim_time_s:.1f}s exceeded. ---")
            break

        # Store velocity before the physics step to detect a stall
        last_velocity = velocity

        current_state = {
            'time': time, 'position': position, 'velocity': velocity,
            'distance_m': distance_m, 'max_accel': max_accel, 'target_v_max': target_v_max
        }

        current_accel = control_law_fn(current_state)
        position, velocity = step_sim(position, velocity, current_accel, dt)
        velocity = max(0, velocity)
        time += dt
        
        # If the controller caused a stall (v=0 after moving), just end the run.
        # This accepts the small undershoot for the sake of a clean plot.
        if velocity == 0 and last_velocity > 0:
            break
        
        t_series.append(time)
        pos_series.append(position)
        vel_series.append(velocity)
        accel_series.append(current_accel)

    return {"time": np.array(t_series), "position": np.array(pos_series), "velocity": np.array(vel_series), "acceleration": np.array(accel_series)}

# =================================================================================
# --- OUTPUT FUNCTIONS ---
# =================================================================================

def calculate_vmax_for_coast_ratio(distance_m, max_accel, coast_ratio):
    """Calculates the target V_max to achieve a desired coast time to total time ratio."""
    # A coast ratio of 1 is a divide-by-zero, and negative is nonsense.
    if not (0 <= coast_ratio < 1):
        return float('nan')
    
    # Derivation: v_max^2 = ad * (1 - R_coast) / (1 + R_coast)
    ad_product = max_accel * distance_m
    scaling_factor = (1 - coast_ratio) / (1 + coast_ratio)
    v_max_squared = ad_product * scaling_factor
    return np.sqrt(v_max_squared)

def get_trapezoidal_time(distance_m, max_accel, v_max):
    """Calculates the minimum time to traverse a distance with a trapezoidal profile, solved analytically."""
    # Velocity reached if the profile is a triangle (accel to midpoint, then decel)
    v_peak_for_distance = np.sqrt(distance_m * max_accel)

    if v_max >= v_peak_for_distance:
        time = 2 * np.sqrt(distance_m / max_accel)
    else:
        time = (distance_m / v_max) + (v_max / max_accel)
    return time

def plot_optimal_speed_tradeoff(paths_m_dict, max_accel, max_force_n, efficiency, voltage):
    """
    Generates a dual-axis plot to visualize the trade-off between trajectory time and
    peak mechanical power for multiple paths.
    """
    plt.style.use('dark_background')
    fig, ax1 = plt.subplots(figsize=(12, 8))
    path_colors = {'Side': 'cyan', 'Diagonal': 'lime', 'Curved': 'magenta'}

    # --- Calculate Y-Limit for Time Axis ---
    # Find the longest path to determine the max time for the y-axis limit
    longest_path_dist = max(paths_m_dict.values())
    # Calculate the time at v=1 m/s for this path to set a reasonable plot ceiling
    y_limit_time = get_trapezoidal_time(longest_path_dist, max_accel, 1.0)

    # --- Time Calculations & Plotting (Left Axis) ---
    v_max_sweep = np.linspace(0.1, 8.0, 400)
    ax1.set_xlabel('Max Velocity Limit (m/s)', fontsize=12)
    ax1.set_ylabel('Total Trajectory Time (s)', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.2)

    for name, distance_m in paths_m_dict.items():
        # Calculate time and the ideal 'knee' velocity for the current path
        times = [get_trapezoidal_time(distance_m, max_accel, v) for v in v_max_sweep]
        v_peak_ideal = np.sqrt(distance_m * max_accel)
        color = path_colors.get(name, 'white') # Use path color, default to white

        # Plot the time curve and its V_peak line
        ax1.plot(v_max_sweep, times, color=color, lw=2.5, label=f'Time ({name})')
        ax1.axvline(x=v_peak_ideal, color=color, linestyle=':', lw=2, label=f'V_peak ({name} = {v_peak_ideal:.2f} m/s)')

    ax1.set_ylim(bottom=0, top=y_limit_time) # Set the new Y-limit

    # --- Power Calculation & Plotting (Right Axis) ---
    powers = [(max_force_n * v) for v in v_max_sweep]

    ax2 = ax1.twinx()
    color2 = 'yellow'
    ax2.set_ylabel('Peak Mechanical Power (W)', color=color2, fontsize=12)
    ax2.plot(v_max_sweep, powers, color=color2, lw=3, linestyle='--', label='Peak Mechanical Power')
    ax2.tick_params(axis='y', labelcolor=color2)
    ax2.set_ylim(bottom=0)

    # --- Final Touches ---
    fig.suptitle('Travel Time and Max Mechanical Power vs Velocity Limit (All Paths)', fontsize=16)
    
    # --- Reorder Legend for Clarity ---
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()

    # Separate ax1 labels into 'Time' and 'V_peak' groups
    time_lines, time_labels = [], []
    vpeak_lines, vpeak_labels = [], []
    for line, label in zip(lines1, labels1):
        if label.startswith('Time'):
            time_lines.append(line)
            time_labels.append(label)
        elif label.startswith('V_peak'):
            vpeak_lines.append(line)
            vpeak_labels.append(label)
            
    # Combine in the desired order: Times, then V_peaks, then Power
    ordered_lines = time_lines + vpeak_lines + lines2
    ordered_labels = time_labels + vpeak_labels + labels2
    
    ax1.legend(ordered_lines, ordered_labels, loc='upper center')
    fig.tight_layout(rect=[0, 0, 1, 0.96])

def plot_vmax_sweep(max_accel):
    """Plots trajectory time vs. max velocity for the different paths."""
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(10, 6))
    path_colors = {'Side': 'cyan', 'Diagonal': 'lime', 'Curved': 'magenta'}

    # Define the sweep range for max velocity
    v_max_sweep = np.linspace(1.0, 8.0, 200) # from 0.1 to 6.0 m/s

    for name, dist_m in PATHS_M.items():
        # Calculate time for each v_max in the sweep using the analytical solution
        times = [get_trapezoidal_time(dist_m, max_accel, v) for v in v_max_sweep]
        ax.plot(v_max_sweep, times, label=f"{name} Path ({dist_m:.2f} m)", color=path_colors[name], lw=2)

    ax.set_xlabel('Max Velocity Limit (m/s)')
    ax.set_ylabel('Total Trajectory Time (s)')
    ax.set_title('Path Times vs. Max Velocity Limit')
    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.set_xlim([1.0, 8.0])
    ax.set_ylim(bottom=0)

def print_static_design_params():
    """Prints a summary of the static design parameters and requirements."""
    print("--------------------------------------")
    print("--- Swerve Design Simulation Setup ---")
    print("--------------------------------------")
    print(f"Robot Mass: {ROBOT_MASS_KG:.2f} kg ({ROBOT_WEIGHT_LBS} lbs)")
    print(f"Max Tractive Force: {MAX_TRACTIVE_FORCE_N:.2f} N")
    print(f"Max Linear Acceleration: {MAX_ACCELERATION_MS2:.2f} m/s^2")
    print(f"Torque per Motor to Slip: {TORQUE_PER_MOTOR_BEFORE_SLIP_NM:.3f} Nm")

def print_power_system_requirements():
    print("---------------------------------")
    print("--- Power System Requirements ---")
    print("---------------------------------")
    print(f"Required Peak Mechanical Power: {P_MECH_PEAK_W:.1f} W (@ {TARGET_V_MAX_MS:.2f}m/s Top Speed, {MAX_TRACTIVE_FORCE_N:.2f}N Pushing Force)")
    print(f"Required Peak Electrical Power: {P_ELEC_PEAK_W:.1f} W (@ {SYSTEM_EFFICIENCY*100:.0f}% eff)")
    print(f"Required Electrical Power per Motor: {P_ELEC_PEAK_W/ NUM_DRIVE_MOTORS:.1f} W (@ {SYSTEM_EFFICIENCY*100:.0f}% eff)")
    print("")

    print(f"Total Drive Current at 4S (@{BATTERY_VOLTAGE_4S}V): {I_TOTAL_REQUIRED_A_4S:.2f} A")
    print(f"Current per Motor at 4S (@{BATTERY_VOLTAGE_4S}V): {I_PER_MOTOR_REQUIRED_A_4S:.2f} A")

    print("")
    print(f"Total Drive Current at 6S (@{BATTERY_VOLTAGE_6S}V): {I_TOTAL_REQUIRED_A_6S:.2f} A")
    print(f"Current per Motor at 6S (@{BATTERY_VOLTAGE_6S}V): {I_PER_MOTOR_REQUIRED_A_6S:.2f} A")
    print("\n")


def print_sim_results(sim_data, title):
    """Prints a summary for a given simulation result."""
    print(f"--- {title} ---")
    header = f"{'Path':<12} | {'Peak Velocity (m/s)':<20} | {'Total Time (s)':<15}"
    print(header)
    print("-" * len(header))
    for name, data in sim_data.items():
        peak_vel = np.max(data['velocity'])
        total_time = data['time'][-1]
        print(f"{name:<12} | {peak_vel:<20.2f} | {total_time:<15.3f}")
    print("-----------------------------------------------------\n")


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

def plot_trajectory_comparison(distance_m, max_accel):
    """
    Plots a comparison of different velocity profiles (min-time vs. trapezoidal)
    for a fixed distance to illustrate the time vs. max_speed trade-off.
    """
    # --- 1. Calculate Baseline and Define V_max Targets ---
    v_peak_ideal = np.sqrt(distance_m * max_accel)
    # Set V_max targets as percentages of the ideal peak
    vmax_75_percent = v_peak_ideal * 0.75
    vmax_50_percent = v_peak_ideal * 0.50
    vmax_25_percent = v_peak_ideal * 0.25

    # --- 2. Run Simulations to Get Trajectory Data ---
    sim_min_time = run_simulation(distance_m, max_accel, control_law_trapezoidal_unlimited)
    time_min = sim_min_time['time'][-1]

    sim_75_trap = run_simulation(distance_m, max_accel, control_law_trapezoidal_limited, target_v_max=vmax_75_percent)
    time_75 = sim_75_trap['time'][-1]

    sim_50_trap = run_simulation(distance_m, max_accel, control_law_trapezoidal_limited, target_v_max=vmax_50_percent)
    time_50 = sim_50_trap['time'][-1]
    
    sim_25_trap = run_simulation(distance_m, max_accel, control_law_trapezoidal_limited, target_v_max=vmax_25_percent)
    time_25 = sim_25_trap['time'][-1]

    # --- 3. Plotting ---
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(12, 7))

    # Plot the min-time profile as a solid baseline
    ax.plot(sim_min_time['time'], sim_min_time['velocity'],
            label=f'Min-Time Profile (V_peak={v_peak_ideal:.2f} m/s) | Total Time: {time_min:.2f}s',
            color='magenta', lw=2.5, linestyle='-')

    # Plot the 75% trapezoid as dashed
    ax.plot(sim_75_trap['time'], sim_75_trap['velocity'],
            label=f'75% V_peak Limit ({vmax_75_percent:.2f} m/s) | Total Time: {time_75:.2f}s',
            color='cyan', lw=2.5, linestyle='--')

    # Plot the 50% trapezoid as dotted
    ax.plot(sim_50_trap['time'], sim_50_trap['velocity'],
            label=f'50% V_peak Limit ({vmax_50_percent:.2f} m/s) | Total Time: {time_50:.2f}s',
            color='lime', lw=2.5, linestyle=':')

    # Plot the 25% trapezoid as dash-dot
    ax.plot(sim_25_trap['time'], sim_25_trap['velocity'],
            label=f'25% V_peak Limit ({vmax_25_percent:.2f} m/s) | Total Time: {time_25:.2f}s',
            color='yellow', lw=2.5, linestyle='-.')

    # --- Final Touches ---
    ax.set_title(f'Example Velocity Profiles', fontsize=16)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Velocity (m/s)', fontsize=12)
    ax.legend(loc='upper right')
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.set_ylim(bottom=0)
    ax.set_xlim(left=0)
    fig.tight_layout()

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
    # Set a symmetric y-axis for acceleration
    max_abs_accel = np.max(np.abs(axes[2, 0].get_ylim()))
    axes[2, 0].set_ylim(-max_abs_accel, max_abs_accel)

def plot_design_trajectories(distance_m, max_accel, design_points):
    """
    Calculates and plots velocity profiles for a list of specified design points,
    with the min-time profile as a baseline.
    """
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(12, 7))
    
    # --- Plot the Min-Time (Triangular) Profile as a baseline ---
    sim_min_time = run_simulation(distance_m, max_accel, control_law_trapezoidal_unlimited)
    ax.plot(sim_min_time['time'], sim_min_time['velocity'],
            color='magenta', lw=2.5, linestyle='--') # Dashed line for baseline

    # --- Plot the sweep of design points ---
    for point in design_points:
        v_max = point['v_max']
        sim_data = run_simulation(distance_m, max_accel, control_law_trapezoidal_limited, target_v_max=v_max)
        
        ax.plot(sim_data['time'], sim_data['velocity'],
                color=point['color'], lw=2.5) # Color comes from the design point

    ax.set_title(f'Velocity Profiles for Various Design Points (Path Distance: {distance_m:.2f}m)', fontsize=16)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Velocity (m/s)', fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.set_ylim(bottom=0)
    ax.set_xlim(left=0)
    fig.tight_layout()

def plot_tradeoff_with_designs(paths_m_dict, max_accel, max_force_n, design_points):
    """
    Generates the Time vs. Power plot with a clean legend and adds markers for design points.
    """
    plt.style.use('dark_background')
    fig, ax1 = plt.subplots(figsize=(12, 8))
    path_colors = {'Side': 'cyan', 'Diagonal': 'lime', 'Curved': 'magenta'}
    v_max_sweep = np.linspace(0.1, 8.0, 400)
    ax1.set_xlabel('Max Velocity Limit (m/s)', fontsize=12)
    ax1.set_ylabel('Total Trajectory Time (s)', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.2)

    # Plot time curves with labels for the legend
    for name, distance_m in paths_m_dict.items():
        times = [get_trapezoidal_time(distance_m, max_accel, v) for v in v_max_sweep]
        ax1.plot(v_max_sweep, times, color=path_colors.get(name, 'white'), lw=2.5, label=f'Travel Time ({name})')
    
    longest_path_dist = max(paths_m_dict.values())
    y_limit_time = get_trapezoidal_time(longest_path_dist, max_accel, 1.0)
    ax1.set_ylim(bottom=0, top=y_limit_time)
    
    ax2 = ax1.twinx()
    powers = [(max_force_n * v) for v in v_max_sweep]
    ax2.set_ylabel('Peak Mechanical Power (W)', color='yellow', fontsize=12)
    # Plot power curve with a label for the legend
    ax2.plot(v_max_sweep, powers, color='yellow', lw=3, linestyle='--', label='Peak Mechanical Power')
    ax2.tick_params(axis='y', labelcolor='yellow')
    ax2.set_ylim(bottom=0)

    # --- Add Markers for the unified design points (unlabeled) ---
    side_path_dist = paths_m_dict.get('Side')
    for point in design_points:
        v_max = point['v_max']
        time_at_vmax = get_trapezoidal_time(side_path_dist, max_accel, v_max)
        power_at_vmax = max_force_n * v_max
        # No 'label' argument for the scatter points
        ax1.scatter(v_max, time_at_vmax, s=80, color=point['color'], zorder=5, ec='white')
        ax2.scatter(v_max, power_at_vmax, s=80, color=point['color'], zorder=5, ec='white')

    # --- Add a special marker for the absolute Min-Time point (unlabeled) ---
    v_peak_ideal = np.sqrt(side_path_dist * max_accel)
    time_at_peak = get_trapezoidal_time(side_path_dist, max_accel, v_peak_ideal)
    power_at_peak = max_force_n * v_peak_ideal
    ax1.scatter(v_peak_ideal, time_at_peak, s=250, color='magenta', marker='*', zorder=6, ec='white')
    ax2.scatter(v_peak_ideal, power_at_peak, s=250, color='magenta', marker='*', zorder=6, ec='white')

    # --- Set new title and build the clean legend for the main curves ---
    fig.suptitle('Travel Time and Peak Power vs. Velocity Limit', fontsize=16)
    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines + lines2, labels + labels2, loc='upper center')
    fig.tight_layout(rect=[0, 0, 1, 0.96])

def calculate_vmax_for_time_ratio(distance_m, max_accel, time_ratio):
    """
    Calculates the target V_max that results in a total time equal to a percentage of the min time.
    """
    if time_ratio < 1.0:
        return float('nan') # Cannot be faster than the minimum time

    # T_min is the time for a triangular profile
    min_time = 2 * np.sqrt(distance_m / max_accel)
    target_time = min_time * time_ratio

    # We need to solve the trapezoidal time equation for v: T = d/v + v/a
    # This rearranges into a quadratic equation: v^2 - (a*T)v + ad = 0
    a_quad = 1
    b_quad = -max_accel * target_time
    c_quad = max_accel * distance_m

    # Check if a solution exists
    discriminant = b_quad**2 - 4 * a_quad * c_quad
    if discriminant < 0:
        return float('nan')

    # The quadratic formula gives two solutions for v. We want the smaller one,
    # which corresponds to the velocity-limited profile on the left side of the time curve's minimum.
    v_max = (-b_quad - np.sqrt(discriminant)) / (2 * a_quad)
    return v_max

def plot_normalized_tradeoff(paths_m_dict, max_accel, max_force_n):
    """
    Generates a 3-row subplot of the normalized trade-off, one for each path,
    with a consistent and correct shared X-axis.
    """
    plt.style.use('dark_background')
    # Create a 3x1 subplot grid, sharing the X-axis for easy comparison.
    fig, axes = plt.subplots(3, 1, figsize=(10, 15), sharex=True)
    
    path_list = sorted(paths_m_dict.items(), key=lambda item: item[1]) # Sort by distance

    fig.suptitle('Normalized Performance Metrics vs. Velocity Limit', fontsize=16)

    # --- Define a single, consistent sweep range for all subplots ---
    # The range is determined by the longest path, which will have the highest v_peak.
    longest_path_dist = max(paths_m_dict.values())
    max_v_peak = np.sqrt(longest_path_dist * max_accel)
    v_max_sweep = np.linspace(0.1, max_v_peak, 500)

    for i, (name, distance_m) in enumerate(path_list):
        ax = axes[i]
        
        # --- Baseline Calculations for this specific path ---
        t_min = 2 * np.sqrt(distance_m / max_accel)
        v_peak_path = np.sqrt(distance_m * max_accel)
        p_at_peak = max_force_n * v_peak_path
        ad_product = max_accel * distance_m

        # --- Sweep Calculations across the shared v_max_sweep ---
        time_penalty_percent = []
        coast_percent = []
        
        for v in v_max_sweep:
            if v > v_peak_path:
                time_penalty_percent.append(0)
                coast_percent.append(0)
            else:
                t_current = get_trapezoidal_time(distance_m, max_accel, v)
                time_penalty_percent.append(((t_current - t_min) / t_min) * 100)
                coast_percent.append(100 * (ad_product - v**2) / (ad_product + v**2) if (ad_product + v**2) > 0 else 0)
        
        powers = [max_force_n * v for v in v_max_sweep]
        power_percent = [(p / p_at_peak) * 100 for p in powers]

        # --- Plotting ---
        ax.plot(v_max_sweep, time_penalty_percent, color='cyan', lw=2.5, linestyle='-', label='Time Penalty (%)')
        ax.plot(v_max_sweep, power_percent, color='yellow', lw=2.5, linestyle='--', label='Power Usage (%)')
        ax.plot(v_max_sweep, coast_percent, color='magenta', lw=2.5, linestyle=':', label='Coast Time (%)')

        # --- Subplot Formatting ---
        ax.set_title(f'Path: {name} ({distance_m:.2f}m)')
        ax.legend(loc='center right')
        ax.grid(True, linestyle='--', alpha=0.3)
        ax.set_ylim(bottom=0, top=100)

    # --- Overall Figure Formatting ---
    axes[-1].set_xlabel('Max Velocity Limit (m/s)', fontsize=12)
    axes[1].set_ylabel('Normalized Percentage (%)', fontsize=12)

    fig.tight_layout(rect=[0, 0.03, 1, 0.96])

def plot_sum_of_squares_cost(paths_m_dict, max_accel, max_force_n):
    """
    Calculates and plots a balanced weighted sum of squares cost function for all paths.
    """
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(12, 7))
    path_colors = {'Side': 'cyan', 'Diagonal': 'lime', 'Curved': 'magenta'}
    
    # Use balanced 50/50 weights for this analysis
    weights = {'w_time': 0.5, 'w_power': 0.5}

    for name, distance_m in paths_m_dict.items():
        # --- Baseline Calculations for this path ---
        t_min = 2 * np.sqrt(distance_m / max_accel)
        v_peak = np.sqrt(distance_m * max_accel)
        p_at_peak = max_force_n * v_peak
        color = path_colors.get(name)

        # --- Sweep and Cost Calculation ---
        v_max_sweep = np.linspace(1.0, v_peak, 400)
        costs = []
        for v in v_max_sweep:
            time_penalty = (get_trapezoidal_time(distance_m, max_accel, v) - t_min) / t_min
            power_usage = (max_force_n * v) / p_at_peak
            total_cost = (weights['w_time'] * (time_penalty**2)) + (weights['w_power'] * (power_usage**2))
            costs.append(total_cost)

        # --- Plotting ---
        ax.plot(v_max_sweep, costs, color=color, lw=2.5, label=f'Cost ({name})')
        min_cost_index = np.argmin(costs)
        optimal_v = v_max_sweep[min_cost_index]
        min_cost = costs[min_cost_index]
        ax.scatter(optimal_v, min_cost, s=150, color=color, ec='white', zorder=5,
                   label=f'Optimal V_max ({name}): {optimal_v:.2f} m/s')

    # --- Final Touches ---
    ax.set_title('Balanced (50/50) Sum of Squares Cost For All Paths', fontsize=16)
    ax.set_xlabel('Max Velocity Limit (m/s)', fontsize=12)
    ax.set_ylabel('Total Weighted Cost (Unitless)', fontsize=12)
    ax.legend(loc='upper right')
    ax.grid(True, linestyle='--', alpha=0.3)
    longest_path_dist = max(paths_m_dict.values())
    max_v_peak = np.sqrt(longest_path_dist * max_accel)
    ax.set_xlim(left=1.0, right=max_v_peak)
    ax.set_ylim(bottom=0)
    fig.tight_layout()

def print_bounds_analysis(paths_m_dict, max_accel, max_force_n):
    """
    Calculates and prints a full suite of performance metrics at a proposed upper-bound velocity.
    """
    # --- Define the Proposed Bounds ---
    LOWER_BOUND_VMAX = 2.0
    UPPER_BOUND_VMAX = 4.0

    print("--- Design Speed Bounds Analysis ---")
    print(f"Proposed Practical Speed Range: {LOWER_BOUND_VMAX:.1f} m/s to {UPPER_BOUND_VMAX:.1f} m/s\n")
    print(f"Performance metrics at the proposed upper bound of {UPPER_BOUND_VMAX:.1f} m/s:")
    
    header = f"{'Path':<12} | {'Time Penalty (%)':<15} | {'Time Penalty (s)':<15} | {'Power Saved (%)':<16} | {'Power Saved (W)':<16} | {'Coast Time (%)':<13} |"
    print(header)
    print("-" * (len(header)))

    # Ensure paths are in a consistent order
    path_list = sorted(paths_m_dict.items(), key=lambda item: item[1])

    for name, dist_m in path_list:
        # --- Time Penalty Calculations ---
        t_min = 2 * np.sqrt(dist_m / max_accel)
        t_at_upper_bound = get_trapezoidal_time(dist_m, max_accel, UPPER_BOUND_VMAX)
        time_penalty_seconds = t_at_upper_bound - t_min
        time_penalty_percent = (time_penalty_seconds / t_min) * 100

        # --- Peak Velocity and Power Calculations ---
        v_peak_path = np.sqrt(max_accel * dist_m)
        v_actual_peak = min(UPPER_BOUND_VMAX, v_peak_path)
        p_max = max_force_n * v_peak_path
        p_actual = max_force_n * v_actual_peak
        power_saved_watts = p_max - p_actual
        percent_power_saved = (power_saved_watts / p_max) * 100 if p_max > 0 else 0

        # --- Coast Percentage Calculation ---
        ad_product = max_accel * dist_m
        if UPPER_BOUND_VMAX < v_peak_path:
            coast_ratio = (ad_product - UPPER_BOUND_VMAX**2) / (ad_product + UPPER_BOUND_VMAX**2)
            coast_percent = coast_ratio * 100
        else:
            coast_percent = 0.0

        print(f"{name:<12} | {time_penalty_percent:>15.2f}% | {time_penalty_seconds:>15.3f}s | {percent_power_saved:>15.2f}% | {power_saved_watts:>15.2f}W | {coast_percent:>13.2f}% |")
    
    print("-" * (len(header)) + "\n")
# =================================================================================
# --- MAIN EXECUTION ---
# =================================================================================

def main():
    """Main function to run the simulation and outputs."""
    try:
        # --- Print static design targets and recommendations ---
        print_static_design_params()
        print_power_system_requirements()
        print_bounds_analysis(PATHS_M, MAX_ACCELERATION_MS2, MAX_TRACTIVE_FORCE_N)

        # --- Simulation runs for detailed analysis ---
        drag_race_sim_data = {}
        for name, dist_m in PATHS_M.items():
            drag_race_sim_data[name] = run_simulation(dist_m, MAX_ACCELERATION_MS2, control_law_drag_race)

        trap_unlimited_sim_data = {}
        for name, dist_m in PATHS_M.items():
            trap_unlimited_sim_data[name] = run_simulation(dist_m, MAX_ACCELERATION_MS2, control_law_trapezoidal_unlimited)
        
        trap_limited_sim_data = {}
        for name, dist_m in PATHS_M.items():
            trap_limited_sim_data[name] = run_simulation(dist_m, MAX_ACCELERATION_MS2, control_law_trapezoidal_limited, target_v_max=TARGET_V_MAX_MS)


        # --- Output simulation results ---
        # print_sim_results(drag_race_sim_data, "Drag Race Simulation Results")
        # print_sim_results(trap_unlimited_sim_data, "Minimum Time Profile Results")
        # print_sim_results(trap_limited_sim_data, f"V-Max Limited Trapezoidal Profile Results (@ {TARGET_V_MAX_MS} m/s)")

        # --- Generate a unified list of design points for plotting ---
        TIME_RATIOS_TO_TEST = np.arange(1.0, 2.0, 0.05)
        
        # --- Generate a curated, 6-color "Cyberpunk" palette ---
        # We'll sample the 'plasma' colormap at 6 specific, non-adjacent points
        # to get a cohesive but high-contrast set of colors.
        colormap = plt.colormaps.get('plasma')
        color_indices = [0.05, 0.25, 0.45, 0.65, 0.85, 0.95]
        CYBERPUNK_COLORS = [colormap(i) for i in color_indices]
        
        design_points = []
        path_for_design = PATHS_M['Diagonal'] # Use Diagonal path as the reference

        for i, ratio in enumerate(TIME_RATIOS_TO_TEST):
            v_max = calculate_vmax_for_time_ratio(path_for_design, MAX_ACCELERATION_MS2, ratio)
            if np.isnan(v_max) or v_max == 0: continue # Skip if no solution
            
            # Cycle through the curated 6-color palette
            color = CYBERPUNK_COLORS[i % len(CYBERPUNK_COLORS)]

            design_points.append({
                'v_max': v_max,
                'label': f'{ratio:.1%} Min Time',
                'color': color
            })
            
        # --- Plotting ---
        if ENABLE_PLOTTING:
            # plot_field_paths()
            
            # Original kinematic plots (useful for diagnostics)
            # plot_kinematics(drag_race_sim_data, title='Kinematic Profiles (Drag Race)')
            # plot_kinematics(trap_unlimited_sim_data, title='Kinematic Profiles (Minimum Time)')

            # Generate the trajectory comparison plot for the Diagonal Path
            # plot_trajectory_comparison(
            #     distance_m=PATHS_M['Diagonal'],
            #     max_accel=MAX_ACCELERATION_MS2
            # )
            # Plot 1: Show what the different profiles look like
            plot_design_trajectories(
                distance_m=path_for_design,
                max_accel=MAX_ACCELERATION_MS2,
                design_points=design_points
            )
            # Plot 2: Show where the design choices land on the trade-off curve
            plot_tradeoff_with_designs(
                paths_m_dict=PATHS_M,
                max_accel=MAX_ACCELERATION_MS2,
                max_force_n=MAX_TRACTIVE_FORCE_N,
                design_points=design_points
            )

            # Plot 3: The new normalized plot for all paths
            plot_normalized_tradeoff(
                paths_m_dict=PATHS_M,
                max_accel=MAX_ACCELERATION_MS2,
                max_force_n=MAX_TRACTIVE_FORCE_N
            )
            # Plot 4: The final, definitive cost function plot for all paths
            plot_sum_of_squares_cost(
                paths_m_dict=PATHS_M,
                max_accel=MAX_ACCELERATION_MS2,
                max_force_n=MAX_TRACTIVE_FORCE_N
            )

            plt.show()

    except KeyboardInterrupt:
        print("\nSimulation aborted by user. Exiting.")
        sys.exit(0)

if __name__ == "__main__":
    main()