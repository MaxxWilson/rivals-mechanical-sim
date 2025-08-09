from motor_data import Motor
import matplotlib.pyplot as plt

# Repeat Mini MK4 Calculated Kv: 3169.84, Vendor Kv: 3500.00
# Repeat Pro Calculated Kv: 2434.71, Vendor Kv: 2500.00
# Repeat Ultra MK2 Calculated Kv: 1367.74, Vendor Kv: 1450.00
# Repeat Compact Calculated Kv: 2190.71, Vendor Kv: 2300.00
# Repeat Max MK2 Calculated Kv: 2058.11, Vendor Kv: 2100.00

V_BAT = 3.3*4

ultra_mk2 = Motor(
    name='Repeat Ultra Mk2',
    v_applied=V_BAT,
    r_phase=0.053,
    kv_rpm=1367.74,
    i_no_load=0.5,
    gear_ratio=14.0
)
ultra_mk2.print_specs()
ultra_mk2.plot_performance_curves()

repeat_max_mk2 = Motor(
    name='Repeat Max Mk2',
    v_applied=V_BAT,
    r_phase=0.182,
    kv_rpm=2058.11,
    i_no_load=0.4,
    gear_ratio=27.0
)
repeat_max_mk2.print_specs()
repeat_max_mk2.plot_performance_curves()

repeat_pro = Motor(
    name='Repeat Pro',
    v_applied=V_BAT,
    r_phase=0.126,
    kv_rpm=2434.71,
    i_no_load=0.4,
    gear_ratio=27.0
)
repeat_pro.print_specs()
repeat_pro.plot_performance_curves()

repeat_compact = Motor(
    name='Repeat Compact',
    v_applied=V_BAT,
    r_phase=0.17,
    kv_rpm=2190.71,
    i_no_load=0.4,
    gear_ratio=22.6
)
repeat_compact.print_specs()
repeat_compact.plot_performance_curves()

repeat_mini_mk4 = Motor(
    name='Repeat Mini Mk4',
    v_applied=V_BAT,
    r_phase=0.32,
    kv_rpm=3169.84,
    i_no_load=0.4,
    gear_ratio=28.5
)
repeat_mini_mk4.print_specs()
repeat_mini_mk4.plot_performance_curves()

plt.show()