import numpy as np
import matplotlib.pyplot as plt

# --- Define Motor Constants ---
# Using the Robomaster M3508 specs at 24V for this example
V_APPLIED = 24.0  # Volts
R_PHASE = 0.194   # Ohms
KT = 0.3          # Nm/A
KV_RPM = 24.48    # RPM/V
I_NO_LOAD = 0.2   # Amps (A reasonable estimate for a motor this size)

# --- Ideal Curve Calculations ---
# 1. Calculate key motor parameters
omega_no_load_rpm = KV_RPM * V_APPLIED
stall_current = V_APPLIED / R_PHASE
tau_stall = KT * stall_current
max_power_watts = (omega_no_load_rpm * 2 * np.pi / 60) * tau_stall / 4

# 2. Generate data points for the curves
speed_rpm = np.linspace(0, omega_no_load_rpm, 500)
speed_rad_s = speed_rpm * (2 * np.pi / 60)

# Calculate torque, current, power, and efficiency for each speed point
torque_nm = tau_stall * (1 - (speed_rpm / omega_no_load_rpm))
current_a = stall_current * (1 - (speed_rpm / omega_no_load_rpm)) + I_NO_LOAD
power_out_watts = torque_nm * speed_rad_s
power_in_watts = V_APPLIED * current_a
# Avoid division by zero at stall for efficiency calculation
efficiency = np.divide(power_out_watts, power_in_watts, out=np.zeros_like(power_out_watts), where=power_in_watts!=0)

# 3. Find peak efficiency point
peak_efficiency_index = np.argmax(efficiency)
peak_efficiency = efficiency[peak_efficiency_index]
power_at_peak_efficiency = power_out_watts[peak_efficiency_index]
speed_at_peak_efficiency = speed_rpm[peak_efficiency_index]


# --- Print Estimated Parameters ---
print("")
print("--- Ideal Motor Parameters ---")
print(f"No-Load Speed: {omega_no_load_rpm:.2f} RPM")
print(f"Stall Current: {stall_current:.2f} A")
print(f"Stall Torque: {tau_stall:.2f} Nm")
print(f"Maximum Power: {max_power_watts:.2f} W")
print(f"Peak Efficiency: {peak_efficiency * 100:.2f} %")
print(f"Power at Peak Efficiency: {power_at_peak_efficiency:.2f} W")
print("----------------------------")


# --- Plotting ---
plt.style.use('dark_background')
fig, ax1 = plt.subplots(figsize=(12, 8))

# Primary Y-Axis (Torque and Current)
ax1.set_xlabel('Speed (RPM)')
ax1.set_ylabel('Torque (Nm) / Current (A)', color='cyan')
ax1.plot(speed_rpm, torque_nm, label='Torque (Nm)', color='cyan')
ax1.plot(speed_rpm, current_a, label='Current (A)', color='lime', linestyle='--')
ax1.tick_params(axis='y', labelcolor='cyan')
ax1.grid(True, linestyle='--', alpha=0.3)

# Secondary Y-Axis (Power and Efficiency)
ax2 = ax1.twinx()
ax2.set_ylabel('Power (W) / Efficiency (%)', color='magenta')
ax2.plot(speed_rpm, power_out_watts, label='Power (W)', color='magenta')
ax2.plot(speed_rpm, efficiency * 100, label='Efficiency (%)', color='yellow', linestyle=':')
# Mark the peak efficiency point
ax2.plot(speed_at_peak_efficiency, peak_efficiency * 100, 'o', ms=8, color='white', label=f'Peak Eff. ({peak_efficiency*100:.1f}%)')
ax2.tick_params(axis='y', labelcolor='magenta')

# Final Touches
ax1.set_title('Ideal Motor Performance Curves')
fig.legend(loc="upper right", bbox_to_anchor=(0.9,0.9), bbox_transform=ax1.transAxes)
fig.tight_layout()
plt.show()