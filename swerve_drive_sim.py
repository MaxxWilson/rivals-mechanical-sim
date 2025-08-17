# swerve_design_sim.py
import numpy as np
import matplotlib.pyplot as plt
import sys

# =================================================================================
# --- GLOBAL CONSTANTS AND ONE-TIME CALCULATIONS ---
# =================================================================================
ENABLE_PLOTTING=True

# --- Base Dimensions (in Inches) ---
FIELD_SIDE_IN = 144.0
ROBOT_DIM_IN = 12.0
BUMPER_THICKNESS_IN = 1.0
TARGET_COAST_RATIO = 0.6

# --- Robot Constants ---
ROBOT_WEIGHT_LBS = 22.5
NUM_DRIVE_MOTORS = 8
WHEEL_DIAMETER_INCHES = 2.75
TARGET_V_MAX_MS = 3.0

# --- Differential Swerve Kinematics ---
# Based on: v_wheel ∝ (v_motor1 - v_motor2), v_steer ∝ (v_motor1 + v_motor2)
# Ratios: 13/44 driving stage 1, 30/14 driving stage 2, 13/44 steering.
# The "/ 2" factor comes from the differential action.
DRIVE_KINEMATIC_CONSTANT_K = (13/44) * (30/14) / 2
STEER_KINEMATIC_CONSTANT_J = (13/44) / 2
# For pure driving, v_m1 = -v_m2. v_wheel = K * (v_m1 - (-v_m1)) = 2*K*v_m1.
# The ratio of motor speed to wheel speed is therefore 1 / (2*K).
TOTAL_DRIVE_RATIO = 1 / (2 * DRIVE_KINEMATIC_CONSTANT_K)


# --- Power System Parameters ---
BATTERY_VOLTAGE_4S = 14.8
BATTERY_VOLTAGE_6S = 22.2
SYSTEM_EFFICIENCY = 0.75 # Estimated efficiency from battery to wheel (85%)

# --- Environment Constants ---
COEFFICIENT_OF_FRICTION = 1.25
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

def print_design_speed_recommendation():
    """Calculates and prints top speed recommendations based on a target coast-time ratio."""
    # --- Design Choice ---
    # Define what percentage of a path's travel time should be spent at max speed.
    # This ensures the chosen V_max is a meaningful part of the trajectory, not just an
    # instantaneous peak. A value of 0.25 means we want to be cruising for 25% of the time.
    
    print("--- Top Speed Design Recommendation (Profile Shape Method) ---")
    print(f"Based on a target 'Coast Time / Total Time' ratio of: {TARGET_COAST_RATIO:.0%}")
    
    header = f"{'Path':<12} | {'Recommended V_max (m/s)':<25}"
    print(header)
    print("-" * (len(header) + 2))
    
    for name, dist_m in PATHS_M.items():
        recommended_vmax = calculate_vmax_for_coast_ratio(dist_m, MAX_ACCELERATION_MS2, TARGET_COAST_RATIO)
        print(f"{name:<12} | {recommended_vmax:<25.2f}")
    print("------------------------------------------\n")

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
    print("--- Swerve Design Simulation Setup ---")
    print(f"Robot Mass: {ROBOT_MASS_KG:.2f} kg ({ROBOT_WEIGHT_LBS} lbs)")
    print(f"Max Tractive Force: {MAX_TRACTIVE_FORCE_N:.2f} N")
    print(f"Target Top Speed: {TARGET_V_MAX_MS:.1f} m/s")
    print(f"Max Linear Acceleration: {MAX_ACCELERATION_MS2:.2f} m/s^2")
    print(f"Acceleration Time From Rest: {TARGET_V_MAX_MS/MAX_ACCELERATION_MS2:.3f} s")
    print(f"Torque per Motor to Slip: {TORQUE_PER_MOTOR_BEFORE_SLIP_NM:.3f} Nm")
    print("-----------------------------------\n")

def print_kinematic_requirements():
    """Prints drivetrain-specific speed requirements."""
    target_wheel_speed_rad_s = TARGET_V_MAX_MS / WHEEL_RADIUS_M
    target_wheel_speed_rpm = target_wheel_speed_rad_s * (60 / (2 * np.pi))
    required_motor_rpm_at_target_v = target_wheel_speed_rpm * TOTAL_DRIVE_RATIO

    print("--- Kinematic & Drivetrain Requirements ---")
    print(f"Total Drive Ratio (Motor:Wheel): {TOTAL_DRIVE_RATIO:.2f}:1")
    print(f"Target Wheel Speed @ {TARGET_V_MAX_MS:.1f} m/s: {target_wheel_speed_rpm:.1f} RPM")
    print(f"Required Motor Speed for V_Max (Pure Drive): {required_motor_rpm_at_target_v:.1f} RPM")
    print("-------------------------------------------\n")


def print_power_system_requirements():
    print("--- Power System Requirements ---")
    print(f"Required Peak Mechanical Power: {P_MECH_PEAK_W:.1f} W (@ {TARGET_V_MAX_MS:.2f}m/s Top Speed, {MAX_TRACTIVE_FORCE_N:.2f}N Pushing Force)")
    print(f"Required Peak Electrical Power: {P_ELEC_PEAK_W:.1f} W (@ {SYSTEM_EFFICIENCY*100:.0f}% eff)")
    print(f"Required Electrical Power per Motor: {P_ELEC_PEAK_W/ NUM_DRIVE_MOTORS:.1f} W (@ {SYSTEM_EFFICIENCY*100:.0f}% eff)")
    print("")

    print(f"Total Drive Current at 4S (@{BATTERY_VOLTAGE_4S}V): {I_TOTAL_REQUIRED_A_4S:.2f} A")
    print(f"Current per Motor at 4S (@{BATTERY_VOLTAGE_4S}V): {I_PER_MOTOR_REQUIRED_A_4S:.2f} A")

    print("")
    print(f"Total Drive Current at 6S (@{BATTERY_VOLTAGE_6S}V): {I_TOTAL_REQUIRED_A_6S:.2f} A")
    print(f"Current per Motor at 6S (@{BATTERY_VOLTAGE_6S}V): {I_PER_MOTOR_REQUIRED_A_6S:.2f} A")
    print("-----------------------------------\n")


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
    Generates the Time vs. Power plot, adds markers for design points, and marks the min-time point.
    """
    plt.style.use('dark_background')
    fig, ax1 = plt.subplots(figsize=(12, 8))
    path_colors = {'Side': 'cyan', 'Diagonal': 'lime', 'Curved': 'magenta'}
    v_max_sweep = np.linspace(0.1, 8.0, 400)
    ax1.set_xlabel('Max Velocity Limit (m/s)', fontsize=12)
    ax1.set_ylabel('Total Trajectory Time (s)', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.2)
    for name, distance_m in paths_m_dict.items():
        times = [get_trapezoidal_time(distance_m, max_accel, v) for v in v_max_sweep]
        ax1.plot(v_max_sweep, times, color=path_colors.get(name, 'white'), lw=2.5) # Removed labels
    
    longest_path_dist = max(paths_m_dict.values())
    y_limit_time = get_trapezoidal_time(longest_path_dist, max_accel, 1.0)
    ax1.set_ylim(bottom=0, top=y_limit_time)
    
    ax2 = ax1.twinx()
    powers = [(max_force_n * v) for v in v_max_sweep]
    ax2.set_ylabel('Peak Mechanical Power (W)', color='yellow', fontsize=12)
    ax2.plot(v_max_sweep, powers, color='yellow', lw=3, linestyle='--') # Removed label
    ax2.tick_params(axis='y', labelcolor='yellow')
    ax2.set_ylim(bottom=0)

    # --- Add Markers for the unified design points ---
    side_path_dist = paths_m_dict.get('Side')
    for point in design_points:
        v_max = point['v_max']
        time_at_vmax = get_trapezoidal_time(side_path_dist, max_accel, v_max)
        power_at_vmax = max_force_n * v_max
        ax1.scatter(v_max, time_at_vmax, s=80, color=point['color'], zorder=5, ec='white')
        ax2.scatter(v_max, power_at_vmax, s=80, color=point['color'], zorder=5, ec='white')

    # --- Add a special marker for the absolute Min-Time point on the Side path ---
    v_peak_ideal = np.sqrt(side_path_dist * max_accel)
    time_at_peak = get_trapezoidal_time(side_path_dist, max_accel, v_peak_ideal)
    power_at_peak = max_force_n * v_peak_ideal
    ax1.scatter(v_peak_ideal, time_at_peak, s=250, color='magenta', marker='*', zorder=6, ec='white')
    ax2.scatter(v_peak_ideal, power_at_peak, s=250, color='magenta', marker='*', zorder=6, ec='white')

    fig.suptitle('Time vs. Power Trade-Off with Various Design Points', fontsize=16)
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

# =================================================================================
# --- MAIN EXECUTION ---
# =================================================================================

def main():
    """Main function to run the simulation and outputs."""
    try:
        # --- Print static design targets and recommendations ---
        print_static_design_params()
        print_kinematic_requirements()
        print_power_system_requirements()
        print_design_speed_recommendation()


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
        TIME_RATIOS_TO_TEST = np.arange(1.0, 2.0, 0.02)
        num_points = len(TIME_RATIOS_TO_TEST)
        
        # --- Generate a high-contrast color list from a sequential colormap ---
        # 1. Create the full sequential colormap
        sequential_colors = plt.cm.plasma(np.linspace(0, 1, num_points))
        
        # 2. Split the colormap into two halves (darks and brights)
        first_half = sequential_colors[:num_points//2]
        second_half = sequential_colors[num_points//2:]
        
        # 3. Weave the two halves together to create an alternating pattern
        contrasted_colors = []
        i, j = 0, 0
        while i < len(first_half) and j < len(second_half):
            contrasted_colors.append(first_half[i])
            i += 1
            contrasted_colors.append(second_half[j])
            j += 1
        # Append any remaining elements if the list has an odd number of points
        contrasted_colors.extend(first_half[i:])
        contrasted_colors.extend(second_half[j:])
        
        # --- Create the design points list ---
        design_points = []
        path_for_design = PATHS_M['Diagonal'] # Use Diagonal path as the reference

        for i, ratio in enumerate(TIME_RATIOS_TO_TEST):
            v_max = calculate_vmax_for_time_ratio(path_for_design, MAX_ACCELERATION_MS2, ratio)
            if np.isnan(v_max) or v_max == 0: continue
            
            design_points.append({
                'v_max': v_max,
                'label': f'{ratio:.1%} Min Time',
                'color': contrasted_colors[i] # Use the new contrasted color
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
            
            plt.show()

    except KeyboardInterrupt:
        print("\nSimulation aborted by user. Exiting.")
        sys.exit(0)

if __name__ == "__main__":
    main()