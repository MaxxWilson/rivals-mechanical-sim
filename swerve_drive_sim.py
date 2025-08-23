# swerve_design_sim.py
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# =================================================================================
# --- GLOBAL CONSTANTS AND ONE-TIME CALCULATIONS ---
# =================================================================================
# Script Control
ENABLE_PLOTTING=False
PRESENTATION_MODE=False

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

def get_trapezoidal_time(distance_m, max_accel, v_max):
    """Calculates the minimum time to traverse a distance with a trapezoidal profile, solved analytically."""
    # Velocity reached if the profile is a triangle (accel to midpoint, then decel)
    v_peak_for_distance = np.sqrt(distance_m * max_accel)

    if v_max >= v_peak_for_distance:
        time = 2 * np.sqrt(distance_m / max_accel)
    else:
        time = (distance_m / v_max) + (v_max / max_accel)
    return time

def print_force_accel_params():
    """Prints a summary of the static design parameters and requirements."""
    print("--------------------------------------")
    print("--- Swerve Design Simulation Setup ---")
    print("--------------------------------------")
    print(f"Robot Mass: {ROBOT_MASS_KG:.2f} kg ({ROBOT_WEIGHT_LBS} lbs)")
    print(f"Max Tractive Force: {MAX_TRACTIVE_FORCE_N:.2f} N")
    print(f"Max Linear Acceleration: {MAX_ACCELERATION_MS2:.2f} m/s^2")

def print_drivetrain_requirements(lower_bound_v, upper_bound_v):
    """
    Calculates and prints the final proposed drivetrain requirements in a summary table.
    """
    # --- Calculations for single-value metrics ---
    force_per_wheel = MAX_TRACTIVE_FORCE_N / NUM_DRIVE_MOTORS

    # --- Calculations for ranged metrics ---
    p_mech_lower = MAX_TRACTIVE_FORCE_N * lower_bound_v
    p_mech_upper = MAX_TRACTIVE_FORCE_N * upper_bound_v
    p_elec_lower = p_mech_lower / SYSTEM_EFFICIENCY
    p_elec_upper = p_mech_upper / SYSTEM_EFFICIENCY
    i_total_4s_lower = p_elec_lower / BATTERY_VOLTAGE_4S
    i_total_4s_upper = p_elec_upper / BATTERY_VOLTAGE_4S
    i_motor_4s_lower = i_total_4s_lower / NUM_DRIVE_MOTORS
    i_motor_4s_upper = i_total_4s_upper / NUM_DRIVE_MOTORS

    # --- Table Formatting ---
    print("\n--- Proposed Drivetrain Requirements ---")
    w1, w2 = 35, 25
    header = f"{'Requirement':<{w1}} | {'Value':<{w2}} |"
    print(header)
    print("-" * len(header))

    # Helper function for printing rows consistently
    def print_row(label, value_str):
        print(f"{label:<{w1}} | {value_str:>{w2}} |")

    # --- Force / Acceleration Section ---
    print_row("Max Acceleration", f"{MAX_ACCELERATION_MS2:.2f} m/s^2")
    print_row("Maximum Pushing Force", f"{MAX_TRACTIVE_FORCE_N:.2f} N")
    print_row("Maximum Force per Wheel", f"{force_per_wheel:.2f} N")
    print("-" * len(header)) # Sub-separator

    # --- Velocity / Power Section ---
    vel_range_str = f"{lower_bound_v:.1f} - {upper_bound_v:.1f} m/s"
    mech_pwr_range_str = f"{p_mech_lower:.1f}W - {p_mech_upper:.1f}W"
    elec_pwr_range_str = f"{p_elec_lower:.1f}W - {p_elec_upper:.1f}W"
    pwr_motor_range_str = f"{(p_elec_lower/NUM_DRIVE_MOTORS):.1f}W - {(p_elec_upper/NUM_DRIVE_MOTORS):.1f}W"

    print_row("Maximum Linear Velocity", vel_range_str)
    print_row("Peak Mechanical Power", mech_pwr_range_str)
    print_row("Peak Electrical Power", elec_pwr_range_str)
    print_row("Power per Motor", pwr_motor_range_str)
    print("-" * len(header)) # Sub-separator

    # --- Current Section (4S) ---
    total_curr_4s_str = f"{i_total_4s_lower:.1f}A - {i_total_4s_upper:.1f}A"
    motor_curr_4s_str = f"{i_motor_4s_lower:.1f}A - {i_motor_4s_upper:.1f}A"

    print_row(f"Total Current (4S @ {BATTERY_VOLTAGE_4S:.1f}V)", total_curr_4s_str)
    print_row("Current per Motor (4S)", motor_curr_4s_str)
    
    print("-" * len(header))

def print_sim_results(sim_data, title):
    """Prints a summary for a given simulation result."""
    print("")
    print(f"--- {title} ---")
    header = f"{'Path':<8} | {'Peak Velocity (m/s)':<20}| {'Total Time (s)':<15}|"
    print(header)
    print("-" * len(header))
    for name, data in sim_data.items():
        peak_vel = np.max(data['velocity'])
        total_time = data['time'][-1]
        print(f"{name:<8} | {peak_vel:>15.2f} m/s | {total_time:>13.3f}s |")
    print("-" * len(header))
    print("")

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

def plot_design_trajectories(distance_m, design_points, max_accel=MAX_ACCELERATION_MS2):
    """
    Calculates and plots velocity profiles for a list of specified design points,
    with the min-time profile as a baseline.
    """
    print("")
    print("Generating Design Trajectories...")
    print("")
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

def plot_tradeoff_with_designs(paths_m_dict, design_points, max_accel=MAX_ACCELERATION_MS2, max_force_n=MAX_TRACTIVE_FORCE_N):
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

def generate_design_points(time_ratios, ref_path_dist, max_accel):
    """
    Generates a list of design points by sweeping through time ratios
    relative to the minimum time for a given reference path.
    """
    colormap = plt.colormaps.get('plasma')
    color_indices = [0.05, 0.25, 0.45, 0.65, 0.85, 0.95]
    cyberpunk_colors = [colormap(i) for i in color_indices]

    design_points = []
    for i, ratio in enumerate(time_ratios):
        v_max = calculate_vmax_for_time_ratio(ref_path_dist, max_accel, ratio)
        if np.isnan(v_max) or v_max == 0:
            continue

        color = cyberpunk_colors[i % len(cyberpunk_colors)]
        design_points.append({
            'ratio': ratio,
            'v_max': v_max,
            'color': color
        })
    return design_points

def print_design_points(design_points, ref_path_name, ref_path_dist, max_accel):
    """
    Prints a professional, full-context table of the generated design points,
    matching the requested right-aligned format.
    """
    # First, calculate the baseline minimum time for the reference path
    t_min = 2 * np.sqrt(ref_path_dist / max_accel)

    print(f"--- Design Point Analysis (Reference Path: {ref_path_name}, Min Time: {t_min:.3f}s) ---")

    # Define column titles and derive widths from them for perfect alignment
    h1, h2, h3 = "Time Ratio (%)", "Time Penalty (s)", "Required V_max (m/s)"
    w1, w2, w3 = len(h1), len(h2), len(h3)

    # Format the header with left-alignment
    header = f"{h1:<{w1}} | {h2:<{w2}} | {h3:<{w3}} |"
    print(header)
    print("-" * len(header))

    for point in design_points:
        ratio = point['ratio']
        v_max = point['v_max']
        
        # Calculate the absolute time penalty in seconds
        time_penalty_s = (ratio * t_min) - t_min
        
        # Build the strings with numbers and their units
        ratio_str = f"{(ratio * 100):.0f}%"
        penalty_str = f"{time_penalty_s:.3f}s"
        vmax_str = f"{v_max:.2f} m/s"
        
        # Print the data strings with right-alignment within the column widths
        print(f"{ratio_str:>{w1}} | {penalty_str:>{w2}} | {vmax_str:>{w3}} |")
        
    print("-" * len(header))

def print_bounds_analysis(paths_m_dict, lower_bound, upper_bound, max_accel=MAX_ACCELERATION_MS2, max_force_n=MAX_TRACTIVE_FORCE_N):
    """
    Calculates and prints a full suite of performance metrics at a proposed upper-bound velocity.
    """
    print("--- Design Speed Bounds Analysis ---")
    print(f"Proposed Practical Speed Range: {lower_bound:.1f} m/s to {upper_bound:.1f} m/s\n")
    print(f"Performance metrics at the proposed upper bound of {upper_bound:.1f} m/s:")
    
    header = f"{'Path':<8} | {'Time Penalty (%)':<15} | {'Time Penalty (s)':<15} | {'Power Saved (%)':<16} | {'Power Saved (W)':<16} | {'Coast Time (%)':<13} |"
    print(header)
    print("-" * (len(header)))

    # Ensure paths are in a consistent order
    path_list = sorted(paths_m_dict.items(), key=lambda item: item[1])

    for name, dist_m in path_list:
        # --- Time Penalty Calculations ---
        t_min = 2 * np.sqrt(dist_m / max_accel)
        t_at_upper_bound = get_trapezoidal_time(dist_m, max_accel, upper_bound)
        time_penalty_seconds = t_at_upper_bound - t_min
        time_penalty_percent = (time_penalty_seconds / t_min) * 100

        # --- Peak Velocity and Power Calculations ---
        v_peak_path = np.sqrt(max_accel * dist_m)
        v_actual_peak = min(upper_bound, v_peak_path)
        p_max = max_force_n * v_peak_path
        p_actual = max_force_n * v_actual_peak
        power_saved_watts = p_max - p_actual
        percent_power_saved = (power_saved_watts / p_max) * 100 if p_max > 0 else 0

        # --- Coast Percentage Calculation ---
        ad_product = max_accel * dist_m
        if upper_bound < v_peak_path:
            coast_ratio = (ad_product - upper_bound**2) / (ad_product + upper_bound**2)
            coast_percent = coast_ratio * 100
        else:
            coast_percent = 0.0

        print(f"{name:<8} | {time_penalty_percent:>15.2f}% | {time_penalty_seconds:>15.3f}s | {percent_power_saved:>15.2f}% | {power_saved_watts:>15.2f}W | {coast_percent:>13.2f}% |")
    
    print("-" * (len(header)))

def print_cost_function_results(paths_m_dict, max_accel, max_force_n):
    """
    Calculates and prints a summary table of the optimal V_max for each path,
    using the same manual formatting as the bounds analysis table.
    """
    print("\n--- Optimal V_max Analysis (Weighted Cost Function) ---")
    print("Finds the V_max where the normalized costs of time and power are balanced.")

    # 1. Define headers with manual, hardcoded widths
    header = f"{'Path':<8} | {'Optimal V_max (m/s)':<19} | {'Time Penalty (s)':<16} | {'Power Saved (%)':<15} |"
    print(header)
    print("-" * len(header))

    weights = {'w_time': 0.5, 'w_power': 0.5}
    path_list = sorted(paths_m_dict.items(), key=lambda item: item[1])

    for name, distance_m in path_list:
        # --- Calculation logic remains the same ---
        t_min = 2 * np.sqrt(distance_m / max_accel)
        v_peak = np.sqrt(distance_m * max_accel)
        p_at_peak = max_force_n * v_peak
        v_max_sweep = np.linspace(1.0, v_peak, 500)
        costs = []
        for v in v_max_sweep:
            time_penalty_norm = (get_trapezoidal_time(distance_m, max_accel, v) - t_min) / t_min
            power_usage_norm = (max_force_n * v) / p_at_peak
            costs.append((weights['w_time'] * (time_penalty_norm**2)) + (weights['w_power'] * (power_usage_norm**2)))
        min_cost_index = np.argmin(costs)
        optimal_v = v_max_sweep[min_cost_index]
        time_at_optimal = get_trapezoidal_time(distance_m, max_accel, optimal_v)
        time_penalty_s = time_at_optimal - t_min
        p_at_optimal = max_force_n * optimal_v
        power_saved_percent = ((p_at_peak - p_at_optimal) / p_at_peak) * 100 if p_at_peak > 0 else 0

        # 2. Format data using manual widths and append units, matching the reference format
        print(f"{name:<8} | {optimal_v:>15.2f} m/s | {time_penalty_s:>15.3f}s | {power_saved_percent:>14.1f}% |")

    print("-" * len(header))

def simulate_all_paths(control_law_fn, **kwargs):
    """Runs a simulation for all standard paths using the provided control law."""
    return {name: run_simulation(dist_m, MAX_ACCELERATION_MS2, control_law_fn, **kwargs) for name, dist_m in PATHS_M.items()}

# -------------------------
# --- Presentation Mode ---
# -------------------------

def presentation_mode_pause(message="Press Enter to continue..."):
    """
    If in presentation mode, pauses execution. It will also manage showing
    and closing any active plot window.
    """
    if not PRESENTATION_MODE:
        return # In non-interactive mode, do nothing and let the script run.

    # --- In Presentation Mode ---
    plt.show(block=False)      # Has no effect if no plot is active
    input(f"--- {message} ---")
    plt.close('all')           # Has no effect if no plot is active
    print("")

def clear_terminal():
    """Clears the terminal screen."""
    # For Windows
    if os.name == 'nt':
        _ = os.system('cls')
    # For macOS and Linux
    else:
        _ = os.system('clear')

# =================================================================================
# --- MAIN EXECUTION ---
# =================================================================================

def main():
    """Main function to run the simulation and outputs."""
    clear_terminal()
    
    try:
        # 1) Max Tractive Force and Linear Acceleration
        print_force_accel_params()
        presentation_mode_pause()

        # 2) Identifying Sample Paths near Field Geometry Limits
        plot_field_paths()
        presentation_mode_pause()

        # 3) Time Optimal 1D Motion Profiling
        drag_race_sim_data = simulate_all_paths(control_law_drag_race)
        plot_kinematics(drag_race_sim_data, title='Kinematic Profiles (Drag Race)')
        print_sim_results(drag_race_sim_data, "Drag Race Simulation Results")
        trap_unlimited_sim_data = simulate_all_paths(control_law_trapezoidal_unlimited)
        plot_kinematics(trap_unlimited_sim_data, title='Kinematic Profiles (Minimum Time)')
        print_sim_results(trap_unlimited_sim_data, "Minimum Time Profile Results")
        presentation_mode_pause()

        # 4) Velocity-Constrained 1D Motion Profiles
        TIME_RATIOS_TO_TEST = np.arange(1.0, 2.0, 0.05)
        path_for_design = PATHS_M['Diagonal']
        design_points = generate_design_points(TIME_RATIOS_TO_TEST, path_for_design, MAX_ACCELERATION_MS2)
        print_design_points(design_points, 'Diagonal', path_for_design, MAX_ACCELERATION_MS2)
        plot_design_trajectories(distance_m=path_for_design, design_points=design_points)
        plot_tradeoff_with_designs(paths_m_dict=PATHS_M, design_points=design_points)
        presentation_mode_pause()

        # 5) Bounding Optimal Top Speed via Design Heuristics
        COURSE_VMAX_LOWER_BOUND = 2.0
        COURSE_VMAX_UPPER_BOUND = 4.0
        plot_normalized_tradeoff(paths_m_dict=PATHS_M, max_accel=MAX_ACCELERATION_MS2, max_force_n=MAX_TRACTIVE_FORCE_N)
        print_bounds_analysis(PATHS_M, lower_bound=COURSE_VMAX_LOWER_BOUND, upper_bound=COURSE_VMAX_UPPER_BOUND)
        presentation_mode_pause()

        # 6) Optimizing Top Speed via Quadratic Cost Minimization
        print_cost_function_results(PATHS_M, MAX_ACCELERATION_MS2, MAX_TRACTIVE_FORCE_N)
        plot_sum_of_squares_cost(paths_m_dict=PATHS_M, max_accel=MAX_ACCELERATION_MS2, max_force_n=MAX_TRACTIVE_FORCE_N)
        presentation_mode_pause()

        # 7)
        TIGHT_VMAX_LOWER_BOUND = 3.0
        TIGHT_VMAX_UPPER_BOUND = 3.5
        print_drivetrain_requirements(lower_bound_v=TIGHT_VMAX_LOWER_BOUND, upper_bound_v=TIGHT_VMAX_UPPER_BOUND)

        plt.show()

    except KeyboardInterrupt:
        print("\nSimulation aborted by user. Exiting.")
        sys.exit(0)

if __name__ == "__main__":
    main()