# main.py
import os
import pandas as pd
import matplotlib.pyplot as plt

import config
from processing.perform_physics_calculations import perform_physics_calculations
from processing.sanitize_idle_current import sanitize_idle_current
from processing.extract_transients import extract_transients
from processing.isolate_step_response import isolate_step_response
from processing.isolate_transient_for_efficiency import isolate_transient_for_efficiency
from plotting.create_main_comparison_plot import create_main_comparison_plot
from plotting.create_efficiency_plot import create_efficiency_plot

if __name__ == "__main__":
    plt.style.use('dark_background')
    
    base_path = os.path.join(os.getcwd(), 'dyno_eval')
    data_directory = os.path.join(base_path, 'ultra_mk2_data')
    
    # --- Select File to Analyze ---
    # Switch this to any of the flywheel test files.
    # file_to_process = 'no_torque_ramp_50A_limit.csv'
    file_to_process = '0.1_torque_ramp_90A_limit.csv'
    # ------------------------------

    full_filepath = os.path.join(data_directory, file_to_process)

    if not os.path.exists(full_filepath):
        print(f"Error: Target file not found at '{full_filepath}'")
    else:
        # 1. LOAD & PROCESS
        print(f"Loading data from {file_to_process}...")
        df = pd.read_csv(full_filepath)
        df[config.COLUMN_MAP['velocity']] = df[config.COLUMN_MAP['velocity']].abs()
        df[config.COLUMN_MAP['current']] = df[config.COLUMN_MAP['current']].abs()
        
        df = sanitize_idle_current(df)
        df = perform_physics_calculations(df)
        transients = extract_transients(df, config.VELOCITY_THRESHOLD)
        
        if not transients:
            print("No transients found to plot.")
        else:
            step_responses = [isolate_step_response(
                t, config.SETTLING_THRESHOLD, config.STEADY_STATE_POINTS_TO_KEEP
            ) for t in transients]
            
            efficiency_transients = [isolate_transient_for_efficiency(t) for t in transients]

            # 2. PLOT
            print("Generating system analysis plots...")
            create_main_comparison_plot(df, step_responses, file_to_process.replace('.csv', ''))
            create_efficiency_plot(efficiency_transients)
            
            plt.show()

        print("Done.")