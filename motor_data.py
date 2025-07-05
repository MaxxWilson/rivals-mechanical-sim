# motor_data.py
import numpy as np
import matplotlib.pyplot as plt

class Motor:
    def __init__(self, name, v_applied, r_phase, kt, kv_rpm, i_no_load=0.2):
        """
        Primary constructor using fundamental physics parameters.
        """
        self.name = name
        self.v_applied = v_applied
        self.r_phase = r_phase
        self.kt = kt
        self.kv_rpm = kv_rpm
        self.i_no_load = i_no_load

        # --- Calculate Derived Specs ---
        self.no_load_speed_rpm = self.kv_rpm * self.v_applied
        self.stall_current = self.v_applied / self.r_phase
        self.stall_torque = self.kt * self.stall_current

    @classmethod
    def from_stall_specs(cls, name, v_applied, stall_torque, stall_current, no_load_speed_rpm, i_no_load=0.2):
        """
        Alternative constructor using vendor-reported performance specs.
        """
        r_phase = v_applied / stall_current
        kt = stall_torque / stall_current
        kv_rpm = no_load_speed_rpm / v_applied
        return cls(name, v_applied, r_phase, kt, kv_rpm, i_no_load)

    def get_performance_curves(self, num_points=500):
        """
        Generates the data for all performance curves. (Internal method)
        """
        speed_rpm = np.linspace(0, self.no_load_speed_rpm, num_points)
        speed_rad_s = speed_rpm * (2 * np.pi / 60)
        torque_nm = self.stall_torque * (1 - (speed_rpm / self.no_load_speed_rpm))
        current_a = self.stall_current * (1 - (speed_rpm / self.no_load_speed_rpm)) + self.i_no_load
        power_out_watts = torque_nm * speed_rad_s
        power_in_watts = self.v_applied * current_a
        efficiency = np.divide(power_out_watts, power_in_watts, out=np.zeros_like(power_out_watts), where=power_in_watts!=0)
        return locals() # Returns a dict of all local variables

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
        print(f"--- {self.name} Parameters ---")
        print(f"No-Load Speed: {self.no_load_speed_rpm:.2f} RPM")
        print(f"Stall Current: {self.stall_current:.2f} A")
        print(f"Stall Torque: {self.stall_torque:.2f} Nm")
        print(f"Maximum Power: {max_power_watts:.2f} W")
        print(f"Peak Efficiency: {peak_efficiency * 100:.2f} %")
        print(f"Power at Peak Efficiency: {power_at_peak_efficiency:.2f} W")
        print("----------------------------")
        print("")

    def plot_performance_curves(self, show=False):
        """
        Generates and displays the performance plot.
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

        ax1.set_xlabel('Speed (RPM)')
        ax1.set_ylabel('Torque (Nm) / Current (A)', color='cyan')
        ax1.plot(speed_rpm, torque_nm, label='Torque (Nm)', color='cyan')
        ax1.plot(speed_rpm, current_a, label='Current (A)', color='lime', linestyle='--')
        ax1.tick_params(axis='y', labelcolor='cyan')
        ax1.grid(True, linestyle='--', alpha=0.3)

        ax2 = ax1.twinx()
        ax2.set_ylabel('Power (W) / Efficiency (%)', color='magenta')
        ax2.plot(speed_rpm, power_out_watts, label='Power (W)', color='magenta')
        ax2.plot(speed_rpm, efficiency * 100, label='Efficiency (%)', color='yellow', linestyle=':')
        ax2.plot(speed_at_peak_efficiency, peak_efficiency_val * 100, 'o', ms=8, color='white', label=f'Peak Eff. ({peak_efficiency_val*100:.1f}%)')
        ax2.tick_params(axis='y', labelcolor='magenta')

        ax1.set_title(f'Ideal Performance Curves: {self.name}')
        fig.legend(loc="upper right", bbox_to_anchor=(0.9,0.9), bbox_transform=ax1.transAxes)
        fig.tight_layout()
        if show:
            plt.show()