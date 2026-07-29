import os
import argparse
from pathlib import Path

# Import your existing scripts as modules
import preprocess
import trajectory_analysis
# import mpc_weight_analysis
import authority_disagreement_analysis
import compare_directories # <-- FIXED IMPORT
import metrics_extraction # <-- NEW IMPORT

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
    # Passes the specific processed directory to calculate metrics and outputs to the mirrored plots dir
    trajectory_analysis.main(
        data_directory=processed_logs_dir, 
        output_directory=os.path.join(plots_base_dir, "trajectory_plots")
    )
    print("")

    print("3. RUNNING METRICS EXTRACTION")
    print("--------------------------------------------------")
    # Extracts Duration, RMSE, and Max Error for this specific run
    metrics_extraction.main(
        data_directory=processed_logs_dir, 
        output_directory=os.path.join(plots_base_dir, "metrics")
    )
    print("")

    print("4. RUNNING AUTHORITY DISAGREEMENT ANALYSIS")
    print("--------------------------------------------------")
    # Calculates control inputs and outputs to the mirrored plots dir
    authority_disagreement_analysis.main(
        data_directory=processed_logs_dir, 
        base_output_dir=os.path.join(plots_base_dir, "authority_plots")
    )
    print("")

    print("==================================================")
    print(f"PIPELINE COMPLETE FOR: {run_name}")
    print(f"Cleaned data saved to:   {os.path.abspath(processed_logs_dir)}")
    print(f"Generated plots saved to: {os.path.abspath(plots_base_dir)}")
    print("==================================================\n")


if __name__ == "__main__":
    # Define the base directory and the path to the main logs folder
    script_base_dir = os.path.dirname(os.path.abspath(__file__))
    logs_base_dir = os.path.join(script_base_dir, "../logs")
    
    # Ensure the logs directory exists before attempting to loop through it
    if not os.path.exists(logs_base_dir):
        print(f"Error: The root logs directory was not found at {os.path.abspath(logs_base_dir)}")
    else:
        # Identify all items in the logs directory that are explicitly folders
        # AND ignore hidden folders (like .pending_questionnaires or .git)
        run_directories = [d for d in os.listdir(logs_base_dir) 
                           if os.path.isdir(os.path.join(logs_base_dir, d)) and not d.startswith('.')]
        
        if not run_directories:
            print(f"No run directories found inside {os.path.abspath(logs_base_dir)}")
        else:
            print(f"Found {len(run_directories)} directories to process. Starting batch analysis...\n")
            
            # Loop through the found directories chronologically/alphabetically and run the pipeline
            for run_dir in sorted(run_directories):
                run_pipeline(run_dir)
                
            print("5. RUNNING CROSS-DIRECTORY COMPARISON ANALYSIS")
            print("--------------------------------------------------")
            # After all individual runs are processed, run the global comparison
            all_processed_logs_dir = os.path.join(script_base_dir, "../processed_logs")
            global_comparison_plots_dir = os.path.join(script_base_dir, "../plots/comparison_plots")
            
            # This calls the actual function inside your compare script
            compare_directories.plot_user_mean_trajectories(
                base_data_dir=all_processed_logs_dir,
                output_dir=global_comparison_plots_dir
            )
            print("")

            print("6. EXTRACTING GLOBAL METRICS SUMMARY")
            print("--------------------------------------------------")
            # Run metrics extraction globally across all processed logs to create a master dataset
            global_metrics_dir = os.path.join(script_base_dir, "../plots/global_metrics")
            metrics_extraction.main(
                data_directory=all_processed_logs_dir,
                output_directory=global_metrics_dir
            )
            print("")
            
            print(">>> ALL RUNS PROCESSED AND COMPARED SUCCESSFULLY! <<<")