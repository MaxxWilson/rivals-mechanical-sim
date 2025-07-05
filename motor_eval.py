from motor_data import Motor
import matplotlib.pyplot as plt

m3508 = Motor(
    name='Robomaster M3508',
    v_applied=24.0,
    r_phase=0.194,
    kt=0.3,
    kv_rpm=24.48,
    i_no_load=0.2
)
m3508.print_specs()
m3508.plot_performance_curves(False)

neo550 = Motor.from_stall_specs(
    name='NEO 550',
    v_applied=12.0,
    stall_torque=0.97,
    stall_current=100,
    no_load_speed_rpm=917*12.0,
    i_no_load=1.4
)
neo550.print_specs()
neo550.plot_performance_curves(False)