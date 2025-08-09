# motor_data.py
import numpy as np
import matplotlib.pyplot as plt

class Motor:
    def __init__(self, name, v_applied, r_phase, kv_rpm, i_no_load=0.2, gear_ratio=1.0):
        """
        Primary constructor using fundamental physics parameters.
        Includes optional external gearing.
        """
        self.name = name
        self.v_applied = v_applied
        self.r_phase = r_phase
        self.kt = 60/(2*np.pi)/kv_rpm
        self.kv_rpm = kv_rpm
        self.i_no_load = i_no_load
        self.gear_ratio = gear_ratio

        # --- Calculate Raw Motor Specs ---
        self.motor_no_load_speed_rpm = self.kv_rpm * self.v_applied
        self.motor_stall_current = self.v_applied / self.r_phase
        self.motor_stall_torque = self.kt * self.motor_stall_current

        # --- Calculate Final Output Specs (Post-Gearing) ---
        self.output_no_load_speed_rpm = self.motor_no_load_speed_rpm / self.gear_ratio
        self.output_stall_torque = self.motor_stall_torque * self.gear_ratio

    @classmethod
    def from_stall_specs(cls, name, v_applied, stall_torque, stall_current, no_load_speed_rpm, i_no_load=0.2, gear_ratio=1.0):
        """
        Alternative constructor using motor-level performance specs.
        """
        r_phase = v_applied / stall_current
        kv_rpm = no_load_speed_rpm / v_applied
        return cls(name, v_applied, r_phase, kv_rpm, i_no_load, gear_ratio)

    def get_performance_curves(self, num_points=500):
        """
        Generates the data for all performance curves based on OUTPUT values.
        """
        speed_rpm = np.linspace(0, self.output_no_load_speed_rpm, num_points)
        torque_nm = self.output_stall_torque * (1 - (speed_rpm / self.output_no_load_speed_rpm))
        
        # Calculate motor-level values to find current and efficiency
        motor_torque_nm = torque_nm / self.gear_ratio
        current_a = (motor_torque_nm / self.kt) + self.i_no_load
        
        power_out_watts = torque_nm * (speed_rpm * 2 * np.pi / 60)
        power_in_watts = self.v_applied * current_a
        
        efficiency = np.divide(power_out_watts, power_in_watts, out=np.zeros_like(power_out_watts), where=power_in_watts!=0)
        return locals() 

    def print_specs(self):
        """
        Calculates and prints the key performance specs.
        """
        curves = self.get_performance_curves()
        power_out_watts = curves['power_out_watts']
        efficiency = curves['efficiency']

        max_power_watts = np.max(power_out_watts)
        peak_efficiency_index = np.argmax(efficiency)
        peak_efficiency = efficiency[peak_efficiency_index]
        power_at_peak_efficiency = power_out_watts[peak_efficiency_index]

        print("")
        print(f"--- {self.name} Parameters (Gear Ratio: {self.gear_ratio}:1) ---")
        print(f"Output No-Load Speed: {self.output_no_load_speed_rpm:.2f} RPM")
        print(f"Motor Stall Current: {self.motor_stall_current:.2f} A")
        print(f"Output Stall Torque: {self.output_stall_torque:.2f} Nm")
        print(f"Maximum Output Power: {max_power_watts:.2f} W")
        print(f"Peak Efficiency: {peak_efficiency * 100:.2f} %")
        print(f"Power at Peak Efficiency: {power_at_peak_efficiency:.2f} W")
        print("----------------------------")
        print("")

    def plot_performance_curves(self, show=False):
        """
        Generates and displays the performance plot based on OUTPUT values.
        """
        curves = self.get_performance_curves()
        speed_rpm = curves['speed_rpm']
        torque_nm = curves['torque_nm']
        current_a = curves['current_a']
        power_out_watts = curves['power_out_watts']
        efficiency = curves['efficiency']
        
        peak_efficiency_index = np.argmax(efficiency)
        speed_at_peak_efficiency = speed_rpm[peak_efficiency_index]
        peak_efficiency_val = efficiency[peak_efficiency_index]

        plt.style.use('dark_background')
        fig, ax1 = plt.subplots(figsize=(12, 8))

        ax1.set_xlabel('Output Speed (RPM)')
        ax1.set_ylabel('Output Torque (Nm) / Current (A)', color='cyan')
        ax1.plot(speed_rpm, torque_nm, label='Output Torque (Nm)', color='cyan')
        ax1.plot(speed_rpm, current_a, label='Motor Current (A)', color='lime', linestyle='--')
        ax1.tick_params(axis='y', labelcolor='cyan')
        ax1.grid(True, linestyle='--', alpha=0.3)

        ax2 = ax1.twinx()
        ax2.set_ylabel('Output Power (W) / Efficiency (%)', color='magenta')
        ax2.plot(speed_rpm, power_out_watts, label='Output Power (W)', color='magenta')
        ax2.plot(speed_rpm, efficiency * 100, label='Efficiency (%)', color='yellow', linestyle=':')
        ax2.plot(speed_at_peak_efficiency, peak_efficiency_val * 100, 'o', ms=8, color='white', label=f'Peak Eff. ({peak_efficiency_val*100:.1f}%)')
        ax2.tick_params(axis='y', labelcolor='magenta')

        ax1.set_title(f'Output Performance Curves: {self.name} ({self.gear_ratio}:1)')
        fig.legend(loc="upper right", bbox_to_anchor=(0.9,0.9), bbox_transform=ax1.transAxes)
        fig.tight_layout()
        if show:
            plt.show()