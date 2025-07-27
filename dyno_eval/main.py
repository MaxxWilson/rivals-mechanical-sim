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
from processing.isolate_transient_for_efficiency import isolate_transient_for_efficiency
from plotting.create_main_comparison_plot import create_main_comparison_plot
from plotting.create_inertia_analysis_plot import create_inertia_analysis_plot
from plotting.create_regression_plot import create_regression_plot
from plotting.create_efficiency_plot import create_efficiency_plot
from exporting.export_inertia_data import export_inertia_data_to_csv
from exporting.export_regression_data import export_regression_data_to_csv

if __name__ == "__main__":
    plt.style.use('dark_background')
    
    base_path = os.path.join(os.getcwd(), 'dyno_eval')
    data_directory = os.path.join(base_path, 'ultra_mk2_data')
    file_to_process = 'no_flywheel.csv'
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
            
            # 2. ANALYZE
            estimated_inertia, regression_plot_data = estimate_motor_inertia(step_responses)
            if estimated_inertia:
                print("-" * 40)
                print(f"FINAL ESTIMATED MOTOR INERTIA: {estimated_inertia:.8f} kg*m^2")
                print("You can update MOTOR_INERTIA in config.py with this value.")
                print("-" * 40)

            # 3. PREPARE & EXPORT DATA
            export_inertia_data_to_csv(step_responses, "inertia_analysis_data", base_path)
            export_regression_data_to_csv(regression_plot_data, "regression_fit_data", base_path)
            efficiency_transients = [isolate_transient_for_efficiency(t) for t in transients]


            # 4. PLOT
            create_main_comparison_plot(df, step_responses, file_to_process.replace('.csv', ''))
            create_inertia_analysis_plot(step_responses, estimated_inertia)
            create_regression_plot(regression_plot_data)
            create_efficiency_plot(efficiency_transients) # This line is now correctly included
            
            plt.show()

        print("Done.")