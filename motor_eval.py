from motor_data import Motor
import matplotlib.pyplot as plt

# The M3508 specs are already for a geared system, so we leave its ratio at 1
m3508 = Motor(
    name='Robomaster M3508',
    v_applied=24.0,
    r_phase=0.194,
    kt=0.3,
    kv_rpm=24.48,
    i_no_load=0.2
)
m3508.print_specs()

# We add a 20:1 external gearbox to the NEO 550
neo550_geared = Motor.from_stall_specs(
    name='NEO 550',
    v_applied=12.0,
    stall_torque=0.97,
    stall_current=100,
    no_load_speed_rpm=11000,
    i_no_load=1.4,
    gear_ratio=1.0 
)
neo550_geared.print_specs()