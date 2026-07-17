import pandas as pd
import argparse
from pathlib import Path

def preprocess_directory(input_dir, output_dir):
    """
    Scans a directory for CSV files, discards data where the study is not running,
    and saves the cleaned data to an output directory.
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    
    # Ensure the input directory exists
    if not input_path.is_dir():
        print(f"Error: The input directory '{input_dir}' does not exist.")
        return
    
    # Create the output directory if it doesn't exist
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Find all CSV files in the input directory
    csv_files = list(input_path.glob("*.csv"))
    
    if not csv_files:
        print(f"No CSV files found in '{input_dir}'.")
        return
    
    print(f"Found {len(csv_files)} CSV file(s). Starting processing...\n")
    
    total_initial_rows = 0
    total_kept_rows = 0
    
    for file_path in csv_files:
        print(f"Processing: {file_path.name}")
        
        try:
            # Read the CSV data
            df = pd.read_csv(file_path)
            initial_rows = len(df)
            total_initial_rows += initial_rows
            
            # Filter rows where the study is running
            if 'study_running' in df.columns:
                filtered_df = df[
                    (df['study_running'] == True) | 
                    (df['study_running'].astype(str).str.strip().str.lower() == 'true')
                ]
            else:
                print(f"  -> Warning: 'study_running' column not found in {file_path.name}. Skipping filtering.")
                filtered_df = df
            
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