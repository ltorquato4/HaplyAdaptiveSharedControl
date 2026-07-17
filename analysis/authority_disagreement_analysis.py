import os
import glob
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 1. Parsing and Mathematical Logic
# ==========================================

def parse_and_calculate_inputs(df):
    """
    Parses the estimated human control parameter (K_h) from JSON strings.
    Calculates the magnitude of human (u_h) and adaptive (u_a) control inputs.
    """
    kh_parsed = []
    
    # 1. Parse K_h
    for json_str in df.get('K_h', []):
        if pd.isna(json_str):
            kh_parsed.append(np.nan)
            continue
        try:
            # Assuming K_h is stored as a JSON array/matrix string 
            # e.g., "[[0.5, 0], [0, 0.5]]" or "[0.5, 0.5]"
            data = json.loads(json_str)
            
            # Extract the primary component (adjust indices if K_h is differently shaped)
            if isinstance(data, list) and len(data) > 0:
                if isinstance(data[0], list): 
                    kh_parsed.append(data[0][0]) # 2D Matrix
                else:
                    kh_parsed.append(data[0])    # 1D Array
            else:
                kh_parsed.append(np.nan)
        except (json.JSONDecodeError, TypeError, IndexError):
            kh_parsed.append(np.nan)

    df['Kh_value'] = kh_parsed

    # 2. Calculate Control Input Magnitudes (|u|)
    # Assumes inputs are logged as vector components (X and Y). 
    # If your data already provides 'u_h' and 'u_a' as magnitudes, it will use those directly.
    
    if 'u_h_x' in df.columns and 'u_h_y' in df.columns:
        df['u_h_mag'] = np.sqrt(df['u_h_x']**2 + df['u_h_y']**2)
    elif 'u_h' in df.columns:
        df['u_h_mag'] = df['u_h'].abs()
    else:
        df['u_h_mag'] = np.nan # Fallback if columns are missing

    if 'u_a_x' in df.columns and 'u_a_y' in df.columns:
        df['u_a_mag'] = np.sqrt(df['u_a_x']**2 + df['u_a_y']**2)
    elif 'u_a' in df.columns:
        df['u_a_mag'] = df['u_a'].abs()
    else:
        df['u_a_mag'] = np.nan

    return df

# ==========================================
# 2. Plotting Functions
# ==========================================

def plot_kh_evolution(trajectories, title_info, filename, output_dir):
    """
    Plots the RLS-estimated Human Control Parameter (K_h) over time.
    """
    plt.figure(figsize=(10, 6))
    
    alpha_val = 1.0 if len(trajectories) == 1 else 0.4

    for df in trajectories:
        # Plot each trajectory's K_h estimate
        if 'Kh_value' in df.columns and not df['Kh_value'].isna().all():
            plt.plot(df['timestamp'], df['Kh_value'], color='purple', alpha=alpha_val)

    plt.title(f"Human Control Parameter ($K_h$) Evolution\n{title_info}")
    plt.xlabel("Timestamp")
    plt.ylabel("Estimated $K_h$ Magnitude")
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename))
    plt.close()


def plot_input_comparison(trajectories, title_info, filename, output_dir):
    """
    Plots the magnitude of human (u_h) vs adaptive (u_a) control inputs 
    to evaluate disagreement (phase/amplitude differences).
    """
    plt.figure(figsize=(12, 6))
    
    alpha_val = 0.8 if len(trajectories) == 1 else 0.3

    for idx, df in enumerate(trajectories):
        # Labels only applied to the first iteration to prevent legend duplication
        label_h = "Human Input ($u_h$)" if idx == 0 else ""
        label_a = "Adaptive Input ($u_a$)" if idx == 0 else ""
        
        if 'u_h_mag' in df.columns:
            plt.plot(df['timestamp'], df['u_h_mag'], color='blue', alpha=alpha_val, label=label_h)
        if 'u_a_mag' in df.columns:
            plt.plot(df['timestamp'], df['u_a_mag'], color='red', alpha=alpha_val, label=label_a)

    plt.title(f"Control Input Comparison ($u_h$ vs. $u_a$)\n{title_info}")
    plt.xlabel("Timestamp")
    plt.ylabel("Control Input Magnitude")
    plt.grid(True)
    plt.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename))
    plt.close()

# ==========================================
# 3. Main Execution Workflow
# ==========================================

def main(data_directory="data", base_output_dir="authority_plots"):
    csv_files = glob.glob(os.path.join(data_directory, "*.csv"))
    
    if not csv_files:
        print("No CSV files found in the specified directory.")
        return

    # 1. Load and pre-process all files
    master_data = []
    for file in csv_files:
        df = pd.read_csv(file)
        df = parse_and_calculate_inputs(df)
        master_data.append(df)
        
    print(f"Successfully loaded and parsed {len(master_data)} trajectories.")

    # Convert to a temporary full dataframe just to extract unique categories
    concat_df = pd.concat(master_data, ignore_index=True)
    controllers = concat_df['controller_type'].dropna().unique()
    behaviors = concat_df['behavior_mode'].dropna().unique()

    # 2. Iterate by Controller Type
    for controller in controllers:
        # Create separate output directories for Fixed and Adaptive
        controller_dir = os.path.join(base_output_dir, controller.lower())
        os.makedirs(controller_dir, exist_ok=True)
        
        # Filter trajectories matching this controller
        ctrl_trajectories = [df for df in master_data if df['controller_type'].iloc[0] == controller]
        
        if not ctrl_trajectories:
            continue
            
        print(f"\nProcessing {controller.upper()} Controller ({len(ctrl_trajectories)} trajectories)...")
        
        # --- Aggregated across ALL behaviors ---
        title_all = f"Controller: {controller.title()} | All Modes"
        plot_kh_evolution(ctrl_trajectories, title_all, f"{controller}_all_modes_Kh.png", controller_dir)
        plot_input_comparison(ctrl_trajectories, title_all, f"{controller}_all_modes_inputs.png", controller_dir)
        
        # --- Separated by distinct Behavior Modes ---
        for behavior in behaviors:
            # Filter trajectories matching this behavior
            beh_trajectories = [
                df for df in ctrl_trajectories 
                if 'behavior_mode' in df.columns and df['behavior_mode'].iloc[0] == behavior
            ]
            
            if not beh_trajectories:
                continue
                
            print(f"  -> Generating plots for behavior mode: '{behavior}'")
            title_beh = f"Controller: {controller.title()} | Mode: {behavior.title()}"
            
            plot_kh_evolution(beh_trajectories, title_beh, f"{controller}_{behavior}_Kh.png", controller_dir)
            plot_input_comparison(beh_trajectories, title_beh, f"{controller}_{behavior}_inputs.png", controller_dir)

    print(f"\nDone. All plots saved in the '{base_output_dir}' directory, sorted by controller type.")

if __name__ == "__main__":
    # Update these directories to point to your specific data locations
    main(data_directory="./trajectories", base_output_dir="./authority_plots")