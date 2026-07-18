import os
import argparse
from pathlib import Path

# Import your existing scripts as modules
import preprocess
import trajectory_analysis
import mpc_weight_analysis
import authority_disagreement_analysis

def run_pipeline(run_name):
    # Define base directories relative to this script
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Map the input directory using the provided run_name (e.g., ../logs/session_1)
    raw_logs_dir = os.path.join(base_dir, "../logs", run_name)
    
    # Map the output directories to mirror the run_name
    processed_logs_dir = os.path.join(base_dir, "../processed_logs", run_name)
    plots_base_dir = os.path.join(base_dir, "../plots", run_name)
    
    print("==================================================")
    print(f"RUNNING PIPELINE FOR: {run_name}")
    print("==================================================\n")
    
    # Verify the target raw logs directory actually exists
    if not os.path.exists(raw_logs_dir):
        print(f"Error: Target directory not found at {os.path.abspath(raw_logs_dir)}")
        return

    print("1. STARTING PREPROCESSING")
    print("--------------------------------------------------")
    # Process the specific session folder and output to the mirrored processed_logs dir
    preprocess.preprocess_directory(
        input_dir=raw_logs_dir, 
        output_dir=processed_logs_dir
    )
    print("")

    print("2. RUNNING TRAJECTORY ANALYSIS")
    print("--------------------------------------------------")
    # Passes the specific processed directory to calculate metrics and outputs to the mirrored plots dir[cite: 4]
    trajectory_analysis.main(
        data_directory=processed_logs_dir, 
        output_directory=os.path.join(plots_base_dir, "trajectory_plots")
    )
    print("")

    print("3. RUNNING MPC WEIGHTS ANALYSIS")
    print("--------------------------------------------------")
    # Parses JSON K_a columns and outputs to the mirrored plots dir[cite: 2]
    mpc_weight_analysis.main(
        data_directory=processed_logs_dir, 
        output_directory=os.path.join(plots_base_dir, "mpc_plots")
    )
    print("")

    print("4. RUNNING AUTHORITY DISAGREEMENT ANALYSIS")
    print("--------------------------------------------------")
    # Calculates control inputs and outputs to the mirrored plots dir[cite: 1]
    authority_disagreement_analysis.main(
        data_directory=processed_logs_dir, 
        base_output_dir=os.path.join(plots_base_dir, "authority_plots")
    )
    print("")

    print("==================================================")
    print("PIPELINE COMPLETE!")
    print(f"Cleaned data saved to:   {os.path.abspath(processed_logs_dir)}")
    print(f"Generated plots saved to: {os.path.abspath(plots_base_dir)}")
    print("==================================================")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Master Execution Script for Specific Runs")
    parser.add_argument(
        "run_name", 
        type=str, 
        help="The name of the directory inside 'logs' to process (e.g., 'session_1')"
    )
    
    args = parser.parse_args()
    run_pipeline(args.run_name)