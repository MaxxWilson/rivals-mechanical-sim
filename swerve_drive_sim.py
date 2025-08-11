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

def run_simulation(distance_m, max_accel, control_law_fn, target_v_max=None, dt=0.0001, max_sim_time_s = 15.0):
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

    # If the requested v_max is higher than what's achievable in a triangular profile for this distance,
    # the profile is limited by acceleration, not velocity. Time is determined by the triangular shape.
    if v_max >= v_peak_for_distance:
        time = 2 * np.sqrt(distance_m / max_accel)
    # Otherwise, the robot hits v_max, coasts, and then decelerates.
    else:
        time = (distance_m / v_max) + (v_max / max_accel)
    return time

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


# =================================================================================
# --- MAIN EXECUTION ---
# =================================================================================

def main():
    """Main function to run the simulation and outputs."""
    try:
        # Print the static parameters first
        print_static_design_params()
        print_kinematic_requirements()
        print_power_system_requirements()


        # --- Run Drag Race Sim ---
        drag_race_sim_data = {}
        for name, dist_m in PATHS_M.items():
            drag_race_sim_data[name] = run_simulation(dist_m, MAX_ACCELERATION_MS2, control_law_drag_race)

        # --- Run Unlimited Trapezoidal Sim ---
        trap_unlimited_sim_data = {}
        for name, dist_m in PATHS_M.items():
            trap_unlimited_sim_data[name] = run_simulation(dist_m, MAX_ACCELERATION_MS2, control_law_trapezoidal_unlimited)

        # --- Run V-Max Limited Trapezoidal Sim ---
        trap_limited_sim_data = {}
        for name, dist_m in PATHS_M.items():
            trap_limited_sim_data[name] = run_simulation(dist_m, MAX_ACCELERATION_MS2, control_law_trapezoidal_limited, target_v_max=TARGET_V_MAX_MS)

        # --- Output Results ---
        print_sim_results(drag_race_sim_data, "Drag Race Simulation Results")
        print_sim_results(trap_unlimited_sim_data, "Minimum Time Profile Results")
        print_sim_results(trap_limited_sim_data, "V-Max Limited Trapezoidal Profile Results")

        # --- Plotting ---
        plot_field_paths()
        plot_kinematics(drag_race_sim_data, title='Kinematic Profiles (Drag Race)')
        plot_kinematics(trap_unlimited_sim_data, title='Kinematic Profiles (Minimum Time)')
        plot_kinematics(trap_limited_sim_data, title=f'Kinematic Profiles (V-Max Limited Trapezoid @ {TARGET_V_MAX_MS} m/s)')
        plot_vmax_sweep(MAX_ACCELERATION_MS2) # New plot call

        if ENABLE_PLOTTING:
            plt.show()

    except KeyboardInterrupt:
        print("\nSimulation aborted by user. Exiting.")
        sys.exit(0)

if __name__ == "__main__":
    main()