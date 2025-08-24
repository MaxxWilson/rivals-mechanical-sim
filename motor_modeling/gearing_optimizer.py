# gearing_optimizer_dual_analysis.py
import numpy as np
import matplotlib.pyplot as plt
from motor_data import Motor

# =================================================================================
# --- BASE SIMULATION & MOTOR CONSTANTS ---
# =================================================================================

V_BAT = 3.7 * 4  # 13.2V

# --- Base values from swerve_drive_sim.py ---
ROBOT_WEIGHT_LBS = 20.0
NUM_DRIVE_MOTORS = 8
WHEEL_DIAMETER_INCHES = 2.75
TARGET_V_MAX_MS = 3.0
COEFFICIENT_OF_FRICTION = 1.2
GRAVITY_MS2 = 9.81
INCHES_TO_METERS = 0.0254

# --- Derived Constants ---
ROBOT_MASS_KG = ROBOT_WEIGHT_LBS * 0.453592
WHEEL_RADIUS_M = (WHEEL_DIAMETER_INCHES * INCHES_TO_METERS) / 2
MAX_TRACTIVE_FORCE_N = ROBOT_MASS_KG * GRAVITY_MS2 * COEFFICIENT_OF_FRICTION
TARGET_WHEEL_SPEED_RPM = (TARGET_V_MAX_MS / WHEEL_RADIUS_M) * (60 / (2 * np.pi))

# --- Motor Definitions ---
motors = [
    Motor(name='Repeat Ultra Mk2', v_applied=V_BAT, r_phase=0.053/2.0, kv_rpm=1367.74, i_no_load=0.5, gear_ratio=14.0),
    Motor(name='Repeat Max Mk2', v_applied=V_BAT, r_phase=0.182/2.0, kv_rpm=2058.11, i_no_load=0.4, gear_ratio=27.0),
    Motor(name='Repeat Pro', v_applied=V_BAT, r_phase=0.126/2.0, kv_rpm=2434.71, i_no_load=0.4, gear_ratio=27.0),
    Motor(name='Repeat Compact', v_applied=V_BAT, r_phase=0.17/2.0, kv_rpm=2190.71, i_no_load=0.4, gear_ratio=22.6),
    Motor(name='Repeat Mini Mk4', v_applied=V_BAT, r_phase=0.32/2.0, kv_rpm=3169.84, i_no_load=0.4, gear_ratio=28.5)
]
motor_map = {m.name: m for m in motors}

# =================================================================================
# --- DATA GATHERING ---
# =================================================================================

z1_range = np.arange(8, 25, 1)
results = {motor.name: {'z1': [], 'headroom': [], 'current': []} for motor in motors}

req_speed_curve = []
req_torque_curve = []

for z1 in z1_range:
    total_drive_ratio = 1 / (2 * (z1/44) * (30/14) / 2)
    required_torque_at_slip = (MAX_TRACTIVE_FORCE_N * WHEEL_RADIUS_M) / (NUM_DRIVE_MOTORS * total_drive_ratio)
    required_speed_at_vmax = TARGET_WHEEL_SPEED_RPM * total_drive_ratio
    req_speed_curve.append(required_speed_at_vmax)
    req_torque_curve.append(required_torque_at_slip)

    for motor in motors:
        if required_torque_at_slip < motor.output_stall_torque:
            speed_at_required_torque = motor.output_no_load_speed_rpm * (1 - (required_torque_at_slip / motor.output_stall_torque))
            if speed_at_required_torque >= required_speed_at_vmax:
                motor_torque_nm = required_torque_at_slip / motor.gear_ratio
                current = (motor_torque_nm / motor.kt) + motor.i_no_load
                results[motor.name]['z1'].append(z1)
                results[motor.name]['headroom'].append(speed_at_required_torque - required_speed_at_vmax)
                results[motor.name]['current'].append(current)

# =================================================================================
# --- OUTPUTS ---
# =================================================================================

print("--- Analysis 1: Optimized for MINIMUM CURRENT ---")
for name, data in results.items():
    if data['z1']:
        idx = np.argmin(data['current'])
        print(f"\n[PASS] {name}: Optimal Z1={data['z1'][idx]} | Min Current={data['current'][idx]:.2f} A | Headroom={data['headroom'][idx]:.1f} RPM")
    else:
        print(f"\n[FAIL] {name}: No valid gearing solution.")
print("\n" + "="*80 + "\n")


print("--- Analysis 2: Optimized for MAXIMUM HEADROOM ---")
for name, data in results.items():
    if data['z1']:
        idx = np.argmax(data['headroom'])
        print(f"\n[PASS] {name}: Optimal Z1={data['z1'][idx]} | Max Headroom={data['headroom'][idx]:.1f} RPM | Current={data['current'][idx]:.2f} A")
    else:
        print(f"\n[FAIL] {name}: No valid gearing solution.")
print("\n" + "="*80 + "\n")


print("--- Raw Motor Power Ranking ---")
power_rankings = sorted(
    [{'name': m.name, 'peak_power_w': m.get_raw_motor_peak_power()} for m in motors],
    key=lambda x: x['peak_power_w'], reverse=True
)
print("| Rank | Motor Name          | Raw Peak Power |")
print("|:----:|:--------------------|:--------------:|")
for i, d in enumerate(power_rankings):
    print(f"| {i+1}    | {d['name']:<19} | {d['peak_power_w']:.2f} W         |")
print("\n" + "="*80 + "\n")


# =================================================================================
# --- PLOTTING ---
# =================================================================================
plt.style.use('dark_background')
fig, ax = plt.subplots(figsize=(14, 9))
ax.plot(z1_range, req_speed_curve, color='red', linestyle='--', linewidth=2, label='Required Speed for V-Max')
ax2 = ax.twinx()
ax2.plot(z1_range, req_torque_curve, color='cyan', linestyle=':', linewidth=2, label='Required Torque to Slip')
motor_colors = plt.cm.viridis(np.linspace(0, 1, len(motors)))
for i, motor in enumerate(motors):
    achievable_speed = [motor.output_no_load_speed_rpm * (1 - (t / motor.output_stall_torque)) if t < motor.output_stall_torque else 0 for t in req_torque_curve]
    ax.plot(z1_range, achievable_speed, color=motor_colors[i], label=f'{motor.name} Achievable Speed')
    ax.fill_between(z1_range, req_speed_curve, achievable_speed,
                    where=[(a >= r) for a, r in zip(achievable_speed, req_speed_curve)],
                    color=motor_colors[i], alpha=0.3, interpolate=True)
ax.set_xlabel('Teeth on Swerve Pinion (Z1)', fontsize=12)
ax.set_ylabel('Gearbox Output Speed (RPM)', color='white', fontsize=12)
ax2.set_ylabel('Gearbox Output Torque (Nm)', color='cyan', fontsize=12)
ax.set_title('Swerve Gearing Optimization', fontsize=16)
ax.grid(True, linestyle='--', alpha=0.2)
ax.set_ylim(bottom=0)
ax2.set_ylim(bottom=0)
lines, labels = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines + lines2, labels + labels2, loc='upper right')
fig.tight_layout()
plt.savefig('gearing_optimization_dual_analysis.png')
print("Generated plot: gearing_optimization_dual_analysis.png\n")