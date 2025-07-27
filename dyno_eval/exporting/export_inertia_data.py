# exporting/export_inertia_data.py
import os
import pandas as pd

def export_inertia_data_to_csv(step_responses, base_filename, base_path):
    """
    Exports the processed step response dataframes to separate CSV files.

    Args:
        step_responses (list): List of DataFrames, where each is a processed step response.
        base_filename (str): The base name for the output CSV files.
        base_path (str): The root path of the 'dyno_eval' directory.
    """
    print(f"-> Exporting {len(step_responses)} transients to CSV...")
    
    # Create the output directory inside 'dyno_eval'
    output_dir = os.path.join(base_path, "analysis_output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    for i, df in enumerate(step_responses):
        filename = f"{base_filename}_transient_{i+1}.csv"
        filepath = os.path.join(output_dir, filename)
        
        df.to_csv(filepath, index=False)
        print(f"   Saved {filepath}")