# motor_eval.py
from motor_data import Motor
import numpy as np

# --- Simulation & Test Constants ---
V_BAT = 3.3*4 # 4S LiPo, 13.2V
# A 5A continuous current limit is a reasonable starting point for 22AWG windings
CONTINUOUS_CURRENT_LIMIT_A = 5.0

# --- Motor Definitions ---
# Using previously measured parameters and applying the current limit
ultra_mk2 = Motor(
    name='Repeat Ultra Mk2',
    v_applied=V_BAT,
    r_phase=0.053/2.0,
    kv_rpm=1367.74,
    i_no_load=0.5,
    gear_ratio=14.0,
    current_limit_a=CONTINUOUS_CURRENT_LIMIT_A
)

repeat_max_mk2 = Motor(
    name='Repeat Max Mk2',
    v_applied=V_BAT,
    r_phase=0.182/2.0,
    kv_rpm=2058.11,
    i_no_load=0.4,
    gear_ratio=27.0,
    current_limit_a=CONTINUOUS_CURRENT_LIMIT_A
)

repeat_pro = Motor(
    name='Repeat Pro',
    v_applied=V_BAT,
    r_phase=0.126/2.0,
    kv_rpm=2434.71,
    i_no_load=0.4,
    gear_ratio=27.0,
    current_limit_a=CONTINUOUS_CURRENT_LIMIT_A
)

repeat_compact = Motor(
    name='Repeat Compact',
    v_applied=V_BAT,
    r_phase=0.17/2.0,
    kv_rpm=2190.71,
    i_no_load=0.4,
    gear_ratio=22.6,
    current_limit_a=CONTINUOUS_CURRENT_LIMIT_A
)

repeat_mini_mk4 = Motor(
    name='Repeat Mini Mk4',
    v_applied=V_BAT,
    r_phase=0.32/2.0,
    kv_rpm=3169.84,
    i_no_load=0.4,
    gear_ratio=28.5,
    current_limit_a=CONTINUOUS_CURRENT_LIMIT_A
)

# --- Analysis ---
motors = [ultra_mk2, repeat_max_mk2, repeat_pro, repeat_compact, repeat_mini_mk4]
power_rankings = []

print(f"\n--- Continuous & Peak Power Analysis (Current Limit: {CONTINUOUS_CURRENT_LIMIT_A:.1f} A) ---\n")

for motor in motors:
    # --- Continuous Performance at Current Limit ---
    speed_at_limit_rpm = motor.speed_at_max_continuous_torque
    speed_at_limit_rads = speed_at_limit_rpm * (2 * np.pi / 60)
    torque_at_limit_nm = motor.max_continuous_output_torque
    continuous_power_w = torque_at_limit_nm * speed_at_limit_rads
    percent_max_speed = (speed_at_limit_rpm / motor.output_no_load_speed_rpm) * 100

    # --- Theoretical Peak Performance (Ignoring Current Limit) ---
    # Peak power occurs at half stall torque and half no-load speed
    peak_power_speed_rads = (motor.output_no_load_speed_rpm * (2 * np.pi / 60)) / 2
    peak_power_torque_nm = motor.output_stall_torque / 2
    peak_power_w = peak_power_torque_nm * peak_power_speed_rads

    power_rankings.append({
        'name': motor.name,
        'cont_power_w': continuous_power_w,
        'peak_power_w': peak_power_w,
        'speed_rpm': speed_at_limit_rpm,
        'percent_max_speed': percent_max_speed
    })

# Sort the list by continuous power in descending order
power_rankings.sort(key=lambda x: x['cont_power_w'], reverse=True)

# --- Print Formatted Results Table ---
w_name, w_cont, w_peak, w_speed, w_pct = 20, 18, 18, 20, 22
header = f"| {'Motor Name':<{w_name}} | {'Cont. Power (W)':<{w_cont}} | {'Peak Power (W)':<{w_peak}} | {'Speed at Limit (RPM)':<{w_speed}} | {'% of Max Speed':<{w_pct}} |"
separator = "-" * len(header)

print(separator)
print(header)
print(separator)

for rank in power_rankings:
    name_str = rank['name']
    cont_p_str = f"{rank['cont_power_w']:.2f}"
    peak_p_str = f"{rank['peak_power_w']:.2f}"
    speed_str = f"{rank['speed_rpm']:.1f}"
    pct_str = f"{rank['percent_max_speed']:.1f}%"
    print(f"| {name_str:<{w_name}} | {cont_p_str:>{w_cont}} | {peak_p_str:>{w_peak}} | {speed_str:>{w_speed}} | {pct_str:>{w_pct}} |")

print(separator)