# motor_data.py
import numpy as np
import matplotlib.pyplot as plt

class Motor:
    def __init__(self, name, v_applied, r_phase, kv_rpm, i_no_load=0.2, gear_ratio=1.0, current_limit_a=None):
        """
        Primary constructor using fundamental physics parameters.
        Includes optional external gearing and a current limit.
        """
        self.name = name
        self.v_applied = v_applied
        self.r_phase = r_phase
        self.kt = 60/(2*np.pi)/kv_rpm
        self.kv_rpm = kv_rpm
        self.i_no_load = i_no_load
        self.gear_ratio = gear_ratio
        self.current_limit_a = current_limit_a if current_limit_a is not None else float('inf')

        # --- Calculate Raw Motor Specs ---
        self.motor_no_load_speed_rpm = self.kv_rpm * self.v_applied
        self.motor_stall_current = self.v_applied / self.r_phase
        self.motor_stall_torque = self.kt * self.motor_stall_current

        # --- Calculate Final Output Specs (Post-Gearing) ---
        self.output_no_load_speed_rpm = self.motor_no_load_speed_rpm / self.gear_ratio
        self.output_stall_torque = self.motor_stall_torque * self.gear_ratio

        # --- Calculate Performance at the Current Limit ---
        self.max_continuous_current = min(self.current_limit_a, self.motor_stall_current)
        self.max_continuous_motor_torque = self.kt * (self.max_continuous_current - self.i_no_load)
        self.max_continuous_output_torque = self.max_continuous_motor_torque * self.gear_ratio
        
        # Ensure max continuous torque is not negative or greater than stall
        self.max_continuous_output_torque = max(0, self.max_continuous_output_torque)
        self.max_continuous_output_torque = min(self.max_continuous_output_torque, self.output_stall_torque)
        
        self.speed_at_max_continuous_torque = self.output_no_load_speed_rpm * (1 - (self.max_continuous_output_torque / self.output_stall_torque))


    @classmethod
    def from_stall_specs(cls, name, v_applied, stall_torque, stall_current, no_load_speed_rpm, i_no_load=0.2, gear_ratio=1.0, current_limit_a=None):
        """
        Alternative constructor using motor-level performance specs.
        """
        r_phase = v_applied / stall_current
        kv_rpm = no_load_speed_rpm / v_applied
        return cls(name, v_applied, r_phase, kv_rpm, i_no_load, gear_ratio, current_limit_a)

    def get_raw_motor_peak_power(self):
        """
        Calculates the theoretical peak mechanical power of the raw, ungeared motor.
        Peak power occurs at 1/2 no-load speed and 1/2 stall torque.
        """
        motor_no_load_speed_rads = self.motor_no_load_speed_rpm * (2 * np.pi / 60)
        peak_power_watts = (self.motor_stall_torque / 2) * (motor_no_load_speed_rads / 2)
        return peak_power_watts

    def get_performance_curves(self, num_points=500):
        """
        Generates the data for all performance curves based on OUTPUT values.
        It now masks the data beyond the specified current limit.
        """
        speed_rpm = np.linspace(0, self.output_no_load_speed_rpm, num_points)
        torque_nm = self.output_stall_torque * (1 - (speed_rpm / self.output_no_load_speed_rpm))
        
        motor_torque_nm = torque_nm / self.gear_ratio
        current_a = (motor_torque_nm / self.kt) + self.i_no_load
        
        power_out_watts = torque_nm * (speed_rpm * 2 * np.pi / 60)
        power_in_watts = self.v_applied * current_a
        
        efficiency = np.divide(power_out_watts, power_in_watts, out=np.zeros_like(power_out_watts), where=power_in_watts!=0)
        
        # --- Create a mask for values beyond the current limit ---
        valid_mask = current_a <= self.current_limit_a
        
        return locals()

    def get_operating_point(self, required_output_torque):
        """
        Calculates the speed and current for a given required output torque.
        Assumes torque is at the final output of the gearbox.
        Now checks against the current limit.
        """
        if required_output_torque > self.output_stall_torque:
            return None  # Cannot produce this torque

        motor_torque_nm = required_output_torque / self.gear_ratio
        current_a = (motor_torque_nm / self.kt) + self.i_no_load

        if current_a > self.current_limit_a:
             return {'error': 'Exceeds current limit'}

        speed_rpm = self.output_no_load_speed_rpm * (1 - (required_output_torque / self.output_stall_torque))
        return {'output_speed_rpm': speed_rpm, 'current_a': current_a}

    def print_specs(self):
        """
        Calculates and prints the key performance specs, including continuous performance.
        """
        curves = self.get_performance_curves()
        power_out_watts = curves['power_out_watts']
        efficiency = curves['efficiency']
        valid_mask = curves['valid_mask']
        
        max_power_watts = np.max(power_out_watts[valid_mask]) if np.any(valid_mask) else 0
        
        # We find peak efficiency within the valid operating region
        valid_efficiency = efficiency[valid_mask]
        peak_efficiency_index_local = np.argmax(valid_efficiency) if np.any(valid_mask) else 0
        original_indices = np.where(valid_mask)[0]
        peak_efficiency_index = original_indices[peak_efficiency_index_local] if original_indices.size > 0 else 0

        peak_efficiency = efficiency[peak_efficiency_index]
        power_at_peak_efficiency = power_out_watts[peak_efficiency_index]

        print("")
        print(f"--- {self.name} Parameters (Gear Ratio: {self.gear_ratio}:1) ---")
        if self.current_limit_a != float('inf'):
            print(f"CONTINUOUS CURRENT LIMIT: {self.current_limit_a:.1f} A")
            print(f"Max Continuous Output Torque: {self.max_continuous_output_torque:.2f} Nm")
            print(f"Speed at Max Continuous Torque: {self.speed_at_max_continuous_torque:.1f} RPM")
        print("--- PEAK PERFORMANCE ---")
        print(f"Output No-Load Speed: {self.output_no_load_speed_rpm:.2f} RPM")
        print(f"Motor Stall Current: {self.motor_stall_current:.2f} A")
        print(f"Output Stall Torque: {self.output_stall_torque:.2f} Nm")
        print(f"Maximum Output Power (within limit): {max_power_watts:.2f} W")
        print(f"Peak Efficiency (within limit): {peak_efficiency * 100:.2f} %")
        print(f"Power at Peak Efficiency: {power_at_peak_efficiency:.2f} W")
        print("----------------------------")
        print("")


    def plot_performance_curves(self, show=False):
        """
        Generates and displays the performance plot.
        Now visually indicates the current limit and its consequences.
        """
        curves = self.get_performance_curves()
        speed_rpm = curves['speed_rpm']
        torque_nm = curves['torque_nm']
        current_a = curves['current_a']
        power_out_watts = curves['power_out_watts']
        efficiency = curves['efficiency']
        valid_mask = curves['valid_mask']
        
        plt.style.use('dark_background')
        fig, ax1 = plt.subplots(figsize=(12, 8))

        # Plot full range lightly
        ax1.plot(speed_rpm, torque_nm, color='cyan', alpha=0.3)
        ax1.plot(speed_rpm, current_a, color='lime', linestyle='--', alpha=0.3)
        ax2 = ax1.twinx()
        ax2.plot(speed_rpm, power_out_watts, color='magenta', alpha=0.3)
        ax2.plot(speed_rpm, efficiency * 100, color='yellow', linestyle=':', alpha=0.3)
        
        # Plot valid range boldly
        ax1.plot(speed_rpm[valid_mask], torque_nm[valid_mask], label='Output Torque (Nm)', color='cyan', lw=2)
        ax1.plot(speed_rpm[valid_mask], current_a[valid_mask], label='Motor Current (A)', color='lime', linestyle='--', lw=2)
        ax2.plot(speed_rpm[valid_mask], power_out_watts[valid_mask], label='Output Power (W)', color='magenta', lw=2)
        ax2.plot(speed_rpm[valid_mask], efficiency[valid_mask] * 100, label='Efficiency (%)', color='yellow', linestyle=':', lw=2)
        
        # Add a vertical line for the limit
        if self.current_limit_a != float('inf'):
            ax1.axvline(x=self.speed_at_max_continuous_torque, color='red', linestyle='-.', lw=2, label=f'Current Limit ({self.current_limit_a} A)')

        # Labels and Titles
        ax1.set_xlabel('Output Speed (RPM)')
        ax1.set_ylabel('Output Torque (Nm) / Current (A)', color='cyan')
        ax1.tick_params(axis='y', labelcolor='cyan')
        ax1.grid(True, linestyle='--', alpha=0.3)
        ax2.set_ylabel('Output Power (W) / Efficiency (%)', color='magenta')
        ax2.tick_params(axis='y', labelcolor='magenta')
        ax1.set_title(f'Performance Curves: {self.name} ({self.gear_ratio}:1)')
        fig.legend(loc="upper right", bbox_to_anchor=(0.9,0.9), bbox_transform=ax1.transAxes)
        fig.tight_layout()
        if show:
            plt.show()