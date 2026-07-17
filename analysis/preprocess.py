import pandas as pd
import argparse
from pathlib import Path

def preprocess_directory(input_dir, output_dir):
    """
    Scans a directory for CSV files, discards non-running study data,
    and normalizes time across all files based on the first file's timestamp.
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    
    if not input_path.is_dir():
        print(f"Error: The input directory '{input_dir}' does not exist.")
        return
    
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Sort files to ensure the "first" file is processed first consistently
    csv_files = sorted(list(input_path.glob("*.csv")))
    
    if not csv_files:
        print(f"No CSV files found in '{input_dir}'.")
        return
    
    print(f"Found {len(csv_files)} CSV file(s). Starting processing...\n")
    
    total_initial_rows = 0
    total_kept_rows = 0
    global_start_time = None
    
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
                ].copy() # .copy() prevents SettingWithCopyWarning when modifying time later
            else:
                print(f"  -> Warning: 'study_running' column not found in {file_path.name}.")
                filtered_df = df.copy()
            
            if filtered_df.empty:
                print("  -> No valid running data found. Skipping.")
                continue

            # Hardcoded check for the 'timestamp' column
            if 'timestamp' not in filtered_df.columns:
                print("  -> Warning: Column 'timestamp' not found. Cannot normalize time.")
            else:
                # Capture the global start time from the very first valid row of the first file
                if global_start_time is None:
                    global_start_time = filtered_df['timestamp'].iloc[0]
                    print(f"  -> [Time Sync] Global start time set to: {global_start_time}")
                
                # Apply the global start time offset to this file
                filtered_df['timestamp'] = filtered_df['timestamp'] - global_start_time
            
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
    if global_start_time is not None:
        print(f"Global Start Time (t=0): {global_start_time}")
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