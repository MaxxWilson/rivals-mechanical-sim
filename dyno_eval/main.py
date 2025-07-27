# main.py
import os
import pandas as pd
import matplotlib.pyplot as plt

# Import our custom modules
import config
from processing.perform_physics_calculations import perform_physics_calculations
from processing.sanitize_idle_current import sanitize_idle_current
from processing.extract_transients import extract_transients
from processing.isolate_step_response import isolate_step_response
from processing.estimate_motor_inertia import estimate_motor_inertia
from plotting.create_main_comparison_plot import create_main_comparison_plot

def run_analysis():
    plt.style.use('dark_background')
    
    data_directory = os.path.join(os.getcwd(), 'ultra_mk2_data')
    file_to_process = 'no_flywheel.csv'
    full_filepath = os.path.join(data_directory, file_to_process)

    if not os.path.exists(full_filepath):
        print(f"Error: Target file not found at '{full_filepath}'")
        return

    # 1. LOAD
    print(f"Loading data from {file_to_process}...")
    df = pd.read_csv(full_filepath)
    df[config.COLUMN_MAP['velocity']] = df[config.COLUMN_MAP['velocity']].abs()
    df[config.COLUMN_MAP['current']] = df[config.COLUMN_MAP['current']].abs()

    # 2. PROCESS
    df = sanitize_idle_current(df)
    df = perform_physics_calculations(df)
    transients = extract_transients(df, config.VELOCITY_THRESHOLD)
    
    if not transients:
        print("No transients found to plot.")
        return

    step_responses = [isolate_step_response(
        t, config.SETTLING_THRESHOLD, config.STEADY_STATE_POINTS_TO_KEEP
    ) for t in transients]
    
    # 3. ANALYZE
    estimated_inertia = estimate_motor_inertia(step_responses)
    if estimated_inertia:
        print("-" * 40)
        print(f"FINAL ESTIMATED MOTOR INERTIA: {estimated_inertia:.8f} kg*m^2")
        print(f"You can update MOTOR_INERTIA in config.py to this value.")
        print("-" * 40)

    # 4. PLOT
    create_main_comparison_plot(df, step_responses, file_to_process.replace('.csv', ''), estimated_inertia)
    plt.show()

    print("Done.")

if __name__ == "__main__":
    run_analysis()