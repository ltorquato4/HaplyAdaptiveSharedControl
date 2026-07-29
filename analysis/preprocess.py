import pandas as pd
import argparse
import re
from collections import defaultdict
from pathlib import Path

def preprocess_directory(input_dir, output_dir):
    """
    Scans a directory for CSV files, filters to keep only the highest attempt 
    for each trial, discards non-running study data, drops uninitialized 
    startup samples, and stitches timestamps together cumulatively per mode.
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    
    if not input_path.is_dir():
        print(f"Error: The input directory '{input_dir}' does not exist.")
        return
        
    output_path.mkdir(parents=True, exist_ok=True)
    
    all_csv_files = list(input_path.glob("*.csv"))
    if not all_csv_files:
        print(f"No CSV files found in '{input_dir}'.")
        return

    # Group files by trial_id and only keep the max attempt_id
    trial_files = defaultdict(list)
    pattern = re.compile(r'trial_(\d+)_attempt_(\d+)')
    
    for file_path in all_csv_files:
        match = pattern.search(file_path.name)
        if match:
            trial_id = int(match.group(1))
            attempt_id = int(match.group(2))
            trial_files[trial_id].append((attempt_id, file_path))
        else:
            # If filename doesn't match the pattern, keep it by default
            trial_files[file_path.name].append((0, file_path))
            
    csv_files = []
    for trial_key, files in trial_files.items():
        # Sort by attempt_id descending and pick the first (highest)
        highest_attempt_file = sorted(files, key=lambda x: x[0], reverse=True)[0][1]
        csv_files.append(highest_attempt_file)
        
    # Sort files to ensure the "first" file is processed first consistently
    csv_files = sorted(csv_files)
    
    print(f"Filtered down to {len(csv_files)} CSV file(s) (highest attempts only). Starting processing...\n")
    
    total_initial_rows = 0
    total_kept_rows = 0
    
    # Track cumulative time independently for each controller mode
    mode_cumulative_time = defaultdict(float)
    
    for file_path in csv_files:
        print(f"Processing: {file_path.name}")
        
        try:
            df = pd.read_csv(file_path)
            initial_rows = len(df)
            total_initial_rows += initial_rows
            
            # Filter rows where the study is running
            if 'study_running' in df.columns:
                filtered_df = df[
                    (df['study_running'] == True) | 
                    (df['study_running'].astype(str).str.strip().str.lower() == 'true')
                ].copy()
            else:
                print(f"  -> Warning: 'study_running' column not found in {file_path.name}.")
                filtered_df = df.copy()
            
            # Drop rows where essential position data is missing (uninitialized state)
            critical_cols = ['cursor_x', 'cursor_y', 'end_x', 'end_y']
            available_critical = [col for col in critical_cols if col in filtered_df.columns]
            if available_critical:
                filtered_df = filtered_df.dropna(subset=available_critical)

            if filtered_df.empty:
                print("  -> No valid running data found after dropping NaNs. Skipping.")
                continue

            # Time normalization per controller mode (Stitching trials together)
            if 'timestamp' not in filtered_df.columns or 'study_controller_mode' not in filtered_df.columns:
                print("  -> Warning: 'timestamp' or 'study_controller_mode' column not found. Cannot normalize time by mode.")
            else:
                # Get the controller mode for this file
                file_mode = filtered_df['study_controller_mode'].astype(str).str.strip().str.lower().iloc[0]
                
                # 1. Normalize this specific trial's timestamps to start at 0.0
                local_time = filtered_df['timestamp'] - filtered_df['timestamp'].iloc[0]
                
                # 2. Shift the normalized time by the cumulative time already spent in this mode
                filtered_df['timestamp'] = local_time + mode_cumulative_time[file_mode]
                
                # 3. Update the cumulative time for the next trial to pick up where this one left off
                # (Adding a tiny 0.01s buffer prevents identical overlapping timestamps)
                mode_cumulative_time[file_mode] = filtered_df['timestamp'].iloc[-1] + 0.01
            
            kept_rows = len(filtered_df)
            total_kept_rows += kept_rows
            
            # Save the preprocessed data to the output directory
            output_file_path = output_path / file_path.name
            filtered_df.to_csv(output_file_path, index=False)
            
            print(f"  -> Kept {kept_rows}/{initial_rows} rows. Saved to {output_file_path}")
            
        except Exception as e:
            print(f"  -> Error processing {file_path.name}: {e}")
            
    print("\n--- Processing Summary ---")
    print(f"Files processed: {len(csv_files)}")
    if mode_cumulative_time:
        print("Total Cumulative Active Time Per Mode:")
        for mode, total_time in mode_cumulative_time.items():
            print(f"  - {mode}: {total_time:.2f} seconds")
    print(f"Total initial rows: {total_initial_rows}")
    print(f"Total discarded rows: {total_initial_rows - total_kept_rows}")
    print(f"Total kept rows: {total_kept_rows}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Batch Preprocess Haply Study Data Directory")
    parser.add_argument(
        "--input_dir", 
        type=str, 
        required=True, 
        help="Path to the directory containing the raw CSV files"
    )
    parser.add_argument(
        "--output_dir", 
        type=str, 
        required=True, 
        help="Path to the directory where cleaned CSV files should be saved"
    )
    
    args = parser.parse_args()
    preprocess_directory(args.input_dir, args.output_dir)