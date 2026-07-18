import os
import glob
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# ==========================================
# 1. Parsing and Mathematical Logic
# ==========================================

def parse_and_calculate_inputs(df):
    kh_parsed = []
    
    for json_str in df.get('K_h', []):
        if pd.isna(json_str):
            kh_parsed.append(np.nan)
            continue
        try:
            data = json.loads(json_str)
            if isinstance(data, list) and len(data) > 0:
                if isinstance(data[0], list): kh_parsed.append(data[0][0])
                else: kh_parsed.append(data[0])
            else:
                kh_parsed.append(np.nan)
        except (json.JSONDecodeError, TypeError, IndexError):
            kh_parsed.append(np.nan)

    df['Kh_value'] = kh_parsed

    def parse_input_array(val):
        if pd.isna(val): return np.nan
        try:
            arr = json.loads(val) if isinstance(val, str) else val
            if isinstance(arr, list) and len(arr) >= 2: return np.sqrt(arr[0]**2 + arr[1]**2)
            elif isinstance(arr, (int, float)): return abs(arr)
            return np.nan
        except: return np.nan
            
    df['u_h_mag'] = df['u_h'].apply(parse_input_array) if 'u_h' in df.columns else np.nan
    df['u_a_mag'] = df['u_a'].apply(parse_input_array) if 'u_a' in df.columns else np.nan

    return df

def get_padded_limits(series_list, pad=0.05):
    """Calculates global min/max limits across multiple columns with a slight padding."""
    valid_mins = [s.min() for s in series_list if s.notna().any()]
    valid_maxs = [s.max() for s in series_list if s.notna().any()]
    if not valid_mins or not valid_maxs: return (0, 1)
    
    min_val = min(valid_mins)
    max_val = max(valid_maxs)
    rng = max_val - min_val
    if rng == 0: return (min_val - 1, max_val + 1)
    
    return (min_val - pad * rng, max_val + pad * rng)

# ==========================================
# 2. Plotting Functions
# ==========================================

def plot_kh_evolution(trajectories, title_info, prefix, output_dir, limits):
    for df in trajectories:
        file_stem = df['file_stem'].iloc[0]
        plt.figure(figsize=(10, 6))
        
        if 'Kh_value' in df.columns and not df['Kh_value'].isna().all():
            plt.plot(df['timestamp'], df['Kh_value'], color='purple')

        plt.title(f"Human Control Parameter ($K_h$) Evolution\n{title_info} | Run: {file_stem}")
        plt.xlabel("Timestamp")
        plt.ylabel("Estimated $K_h$ Magnitude")
        plt.xlim(limits['time'])
        plt.ylim(limits['kh'])
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f"{prefix}_{file_stem}_Kh.pdf"))
        plt.close()

def plot_input_comparison(trajectories, title_info, prefix, output_dir, limits):
    for df in trajectories:
        file_stem = df['file_stem'].iloc[0]
        plt.figure(figsize=(12, 6))
        
        if 'u_h_mag' in df.columns:
            plt.plot(df['timestamp'], df['u_h_mag'], color='blue', label="Human Input ($u_h$)")
        if 'u_a_mag' in df.columns:
            plt.plot(df['timestamp'], df['u_a_mag'], color='red', label="Adaptive Input ($u_a$)")

        plt.title(f"Control Input Comparison ($u_h$ vs. $u_a$)\n{title_info} | Run: {file_stem}")
        plt.xlabel("Timestamp")
        plt.ylabel("Control Input Magnitude")
        plt.xlim(limits['time'])
        plt.ylim(limits['u_mag'])
        plt.grid(True)
        plt.legend(loc='upper right')
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f"{prefix}_{file_stem}_inputs.pdf"))
        plt.close()

# ==========================================
# 3. Main Execution Workflow
# ==========================================

def main(data_directory="data", base_output_dir="authority_plots"):
    csv_files = glob.glob(os.path.join(data_directory, "**", "*.csv"), recursive=True)
    
    if not csv_files:
        print("No CSV files found in the specified directory.")
        return

    master_data = []
    for file in csv_files:
        df = pd.read_csv(file)
        if 'study_controller_mode' in df.columns: df['study_controller_mode'] = df['study_controller_mode'].astype(str).str.lower()
        if 'study_phase' in df.columns: df['study_phase'] = df['study_phase'].astype(str).str.lower()
            
        df = parse_and_calculate_inputs(df)
        df['file_stem'] = Path(file).stem
        master_data.append(df)
        
    concat_df = pd.concat(master_data, ignore_index=True)
    
    # --- CALCULATE GLOBAL LIMITS FOR SCALING ---
    limits = {
        'time': get_padded_limits([concat_df['timestamp']], pad=0),
        'kh': get_padded_limits([concat_df['Kh_value']]),
        'u_mag': get_padded_limits([concat_df['u_h_mag'], concat_df['u_a_mag']])
    }

    controllers = concat_df['study_controller_mode'].dropna().unique()
    phases = concat_df['study_phase'].dropna().unique()

    for controller in controllers:
        ctrl_trajectories = [df for df in master_data if df['study_controller_mode'].iloc[0] == controller]
        if not ctrl_trajectories: continue
        print(f"\nProcessing scaled plots for {controller.upper()} Controller...")
        
        for phase in phases:
            phase_trajectories = [df for df in ctrl_trajectories if 'study_phase' in df.columns and df['study_phase'].iloc[0] == phase]
            if not phase_trajectories: continue
            
            phase_dir = os.path.join(base_output_dir, controller, phase)
            os.makedirs(phase_dir, exist_ok=True)
            title_beh = f"Controller: {controller.title()} | Phase: {phase.title()}"
            
            plot_kh_evolution(phase_trajectories, title_beh, f"{controller}_{phase}", phase_dir, limits)
            plot_input_comparison(phase_trajectories, title_beh, f"{controller}_{phase}", phase_dir, limits)

    print(f"\nDone. All scaled individual plots saved in '{base_output_dir}'.")

if __name__ == "__main__":
    main(data_directory="../processed_logs", base_output_dir="../plots/authority_plots")