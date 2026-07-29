import os
import glob
import numpy as np
import pandas as pd
from pathlib import Path

# ==========================================
# 1. Metric Calculations
# ==========================================

def calculate_trial_metrics(df, file_stem):
    """Calculates duration, RMSE, and maximum error for a single trial."""
    
    # 1. Calculate Orthogonal (Cross-Track) Error
    line_vec_x = df['end_x'] - df['start_x']
    line_vec_y = df['end_y'] - df['start_y']
    line_len_sq = line_vec_x**2 + line_vec_y**2
    line_len = np.sqrt(line_len_sq)

    point_vec_x = df['cursor_x'] - df['start_x']
    point_vec_y = df['cursor_y'] - df['start_y']

    # Absolute orthogonal error (magnitude of deviation)
    cross_prod = np.abs(point_vec_x * line_vec_y - point_vec_y * line_vec_x)
    orthogonal_error = np.where(line_len == 0, 0, cross_prod / line_len)
    
    # 2. Extract Study Data Metadata safely
    mode = df['study_controller_mode'].iloc[0] if 'study_controller_mode' in df.columns else 'unknown'
    phase = df['study_phase'].iloc[0] if 'study_phase' in df.columns else 'unknown'
    
    # 3. Calculate Core Metrics matching study_analysis logic
    duration_s = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]
    cross_track_rmse = np.sqrt(np.mean(orthogonal_error**2))
    cross_track_max = np.max(orthogonal_error)
    
    return {
        'file_name': file_stem,
        'controller_mode': str(mode).strip().lower(),
        'phase': str(phase).strip().lower(),
        'duration_s': float(duration_s),
        'cross_track_rmse': float(cross_track_rmse),
        'cross_track_max': float(cross_track_max)
    }

# ==========================================
# 2. Main Execution Workflow
# ==========================================

def main(data_directory="../processed_logs", output_directory="../plots/metrics"):
    os.makedirs(output_directory, exist_ok=True)
    csv_files = glob.glob(os.path.join(data_directory, "**", "*.csv"), recursive=True)
    
    if not csv_files:
        print(f"No CSV files found in {data_directory}")
        return

    metrics_list = []
    
    for file in csv_files:
        try:
            df = pd.read_csv(file)
            
            # SAFETY CHECK: Ensure critical columns exist before processing
            required_cols = ['timestamp', 'end_x', 'start_x', 'end_y', 'start_y', 'cursor_x', 'cursor_y']
            if not all(col in df.columns for col in required_cols):
                print(f"Skipping {Path(file).name}: Missing required columns.")
                continue
            
            # Drop uninitialized rows
            df = df.dropna(subset=required_cols).reset_index(drop=True)
            
            if len(df) < 2:
                print(f"Skipping {Path(file).name}: Not enough valid samples.")
                continue
                
            file_stem = Path(file).stem
            metrics = calculate_trial_metrics(df, file_stem)
            metrics_list.append(metrics)
            
        except pd.errors.EmptyDataError:
            print(f"Skipping empty file: {file}")
        except Exception as e:
            print(f"Error processing {file}: {e}")
            
    if not metrics_list:
        print("No valid metrics could be calculated.")
        return

    # Convert list of dictionaries to a DataFrame
    metrics_df = pd.DataFrame(metrics_list)
    
    # Save to CSV
    output_path = os.path.join(output_directory, "trial_metrics_summary.csv")
    metrics_df.to_csv(output_path, index=False)
    print(f"Successfully calculated metrics for {len(metrics_df)} trials.")
    print(f"Summary saved to: {output_path}")
    
    # Optionally print a quick condition summary to the terminal
    print("\n--- Quick Condition Summary (Mean) ---")
    summary = metrics_df.groupby(['controller_mode', 'phase']).mean(numeric_only=True)
    print(summary)

if __name__ == "__main__":
    main()