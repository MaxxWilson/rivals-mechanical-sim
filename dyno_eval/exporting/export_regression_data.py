# exporting/export_regression_data.py
import os

def export_regression_data_to_csv(plot_data, base_filename, base_path):
    if not plot_data: return
    print(f"-> Exporting regression dataset to CSV...")
    output_dir = os.path.join(base_path, "analysis_output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    inlier_filepath = os.path.join(output_dir, f"{base_filename}_inliers.csv")
    plot_data['inliers'].to_csv(inlier_filepath, index=False)
    print(f"   Saved {inlier_filepath}")
    if not plot_data['outliers'].empty:
        outlier_filepath = os.path.join(output_dir, f"{base_filename}_outliers.csv")
        plot_data['outliers'].to_csv(outlier_filepath, index=False)
        print(f"   Saved {outlier_filepath}")