# motor_eval.py
from motor_data import Motor
import matplotlib.pyplot as plt

# --- Simulation Requirements ---
# These values are taken directly from the swerve_drive_sim.py output
REQUIRED_TORQUE_AT_SLIP_NM = 0.295
REQUIRED_SPEED_AT_VMAX_RPM = 1295.6
V_BAT = 3.3*4 # 4S LiPo

# --- Motor Definitions ---
# Using calculated Kv values for higher fidelity
ultra_mk2 = Motor(
    name='Repeat Ultra Mk2',
    v_applied=V_BAT,
    r_phase=0.053,
    kv_rpm=1367.74,
    i_no_load=0.5,
    gear_ratio=14.0
)

repeat_max_mk2 = Motor(
    name='Repeat Max Mk2',
    v_applied=V_BAT,
    r_phase=0.182,
    kv_rpm=2058.11,
    i_no_load=0.4,
    gear_ratio=27.0
)

repeat_pro = Motor(
    name='Repeat Pro',
    v_applied=V_BAT,
    r_phase=0.126,
    kv_rpm=2434.71,
    i_no_load=0.4,
    gear_ratio=27.0
)

repeat_compact = Motor(
    name='Repeat Compact',
    v_applied=V_BAT,
    r_phase=0.17,
    kv_rpm=2190.71,
    i_no_load=0.4,
    gear_ratio=22.6
)

repeat_mini_mk4 = Motor(
    name='Repeat Mini Mk4',
    v_applied=V_BAT,
    r_phase=0.32,
    kv_rpm=3169.84,
    i_no_load=0.4,
    gear_ratio=28.5
)

# --- Analysis ---
motors = [ultra_mk2, repeat_max_mk2, repeat_pro, repeat_compact, repeat_mini_mk4]

print(f"\n--- Analysis for Required Traction-Limited Torque: {REQUIRED_TORQUE_AT_SLIP_NM:.3f} Nm ---")
print(f"--- Required speed for V_max: {REQUIRED_SPEED_AT_VMAX_RPM:.1f} RPM ---\n")

for motor in motors:
    motor.print_specs()
    op_point = motor.get_operating_point(REQUIRED_TORQUE_AT_SLIP_NM)

    print(f"--- {motor.name} Traction Performance ---")
    if op_point:
        speed_rpm = op_point['output_speed_rpm']
        current_a = op_point['current_a']
        print(f"    Speed at Required Torque: {speed_rpm:.1f} RPM")
        print(f"    Current at Required Torque: {current_a:.2f} A")
        
        if speed_rpm < REQUIRED_SPEED_AT_VMAX_RPM:
             print(f"    \033[91mWARNING:\033[0m Speed at slip torque ({speed_rpm:.1f}) is BELOW required V_max speed ({REQUIRED_SPEED_AT_VMAX_RPM:.1f}).")
        else:
             print(f"    \033[92mPASS:\033[0m Speed headroom above V_max requirement: {speed_rpm - REQUIRED_SPEED_AT_VMAX_RPM:.1f} RPM")

    else:
        print(f"    \033[91mFAIL:\033[0m MOTOR CANNOT PRODUCE REQUIRED TORQUE. (Stall: {motor.output_stall_torque:.3f} Nm)")
    print("------------------------------------------\n")

# --- Plotting ---
for motor in motors:
    motor.plot_performance_curves()

plt.show()