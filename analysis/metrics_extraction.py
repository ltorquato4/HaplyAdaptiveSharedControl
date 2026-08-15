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
    
    # Extract Participant ID (Checks for column first, falls back to filename prefix)
    if 'participant_id' in df.columns:
        participant = str(df['participant_id'].iloc[0])
    elif 'subject_id' in df.columns:
        participant = str(df['subject_id'].iloc[0])
    else:
        # Assumes format like "P01_trial_data.csv" -> extracts "P01"
        participant = file_stem.split('_')[0] 
    
    # 3. Calculate Core Metrics
    duration_s = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]
    cross_track_rmse = np.sqrt(np.mean(orthogonal_error**2))
    cross_track_max = np.max(orthogonal_error)
    
    return {
        'file_name': file_stem,
        'participant_id': participant,
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
    print(f"Successfully calculated metrics for {len(metrics_df)} total trials.\n")
    
    # ---------------------------------------------------------
    # 3. Create Single Summary DataFrame for Table Output
    # ---------------------------------------------------------
    summary_data = []

    # --- A) Group by Controller + Phase ---
    # Step 1: Sum duration and average errors PER participant
    part_phase_group = metrics_df.groupby(['controller_mode', 'phase', 'participant_id']).agg({
        'duration_s': 'sum',
        'cross_track_rmse': 'mean',
        'cross_track_max': 'mean'
    }).reset_index()

    # Step 2: Average those values ACROSS all participants
    for (controller, phase), group_df in part_phase_group.groupby(['controller_mode', 'phase']):
        summary_data.append({
            'Controller': controller.capitalize(),
            'Mode': phase.capitalize(),
            'Duration': group_df['duration_s'].mean(),
            'RMSE': group_df['cross_track_rmse'].mean(),
            'Maximum Error': group_df['cross_track_max'].mean()
        })

    # --- B) Group by Controller Only (for the 'All' phase row) ---
    # Step 1: Sum duration and average errors PER participant across all phases
    part_all_group = metrics_df.groupby(['controller_mode', 'participant_id']).agg({
        'duration_s': 'sum',
        'cross_track_rmse': 'mean',
        'cross_track_max': 'mean'
    }).reset_index()

    # Step 2: Average those total values ACROSS all participants
    for controller, group_df in part_all_group.groupby('controller_mode'):
        summary_data.append({
            'Controller': controller.capitalize(),
            'Mode': 'All',
            'Duration': group_df['duration_s'].mean(), 
            'RMSE': group_df['cross_track_rmse'].mean(),
            'Maximum Error': group_df['cross_track_max'].mean()
        })

    # Create DataFrame from the summary data
    summary_df = pd.DataFrame(summary_data)

    # Sort to match the desired table format (Fixed -> Adaptive; Careful -> Normal -> Aggressive -> All)
    mode_order = {'Careful': 0, 'Normal': 1, 'Aggressive': 2, 'All': 3}
    summary_df['mode_sort'] = summary_df['Mode'].map(mode_order)
    
    # Sorts 'Fixed' before 'Adaptive' by using descending order for the Controller column
    summary_df = summary_df.sort_values(by=['Controller', 'mode_sort'], ascending=[False, True]).drop(columns=['mode_sort'])

    # Export to a single CSV file
    filename = "performance_measures_summary.csv"
    output_path = os.path.join(output_directory, filename)
    summary_df.to_csv(output_path, index=False)
    print(f" -> Saved aggregated summary metrics file to: {output_path}")

if __name__ == "__main__":
    main()