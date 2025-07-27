# exporting/export_inertia_data.py
import os

def export_inertia_data_to_csv(step_responses, base_filename, base_path):
    print(f"-> Exporting {len(step_responses)} transients to CSV...")
    output_dir = os.path.join(base_path, "analysis_output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    for i, df in enumerate(step_responses):
        filename = f"{base_filename}_transient_{i+1}.csv"
        filepath = os.path.join(output_dir, filename)
        df.to_csv(filepath, index=False)
        print(f"   Saved {filepath}")