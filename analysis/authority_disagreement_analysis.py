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

def generate_authority_plots(df, controller, behavior, output_dir, limits, aggregate_only=False):
    save_dir = os.path.join(output_dir, controller, behavior)
    os.makedirs(save_dir, exist_ok=True)
    
    trajectories = df['file_stem'].unique()
    prefix = f"{controller}_{behavior}"
    title_info = f"Controller: {controller.title()} | Phase: {behavior.replace('_', ' ').title()}"

    # ----------------------------------------
    # INDIVIDUAL PLOTS
    # ----------------------------------------
    if not aggregate_only:
        for traj in trajectories:
            traj_data = df[df['file_stem'] == traj]
            
            # --- Plot 1: Kh Evolution ---
            plt.figure(figsize=(10, 6))
            if 'Kh_value' in traj_data.columns and not traj_data['Kh_value'].isna().all():
                plt.plot(traj_data['timestamp'], traj_data['Kh_value'], color='purple')
            plt.title(f"Human Control Parameter ($K_h$) Evolution\n{title_info} | Run: {traj}")
            plt.xlabel("Timestamp")
            plt.ylabel("Estimated $K_h$ Magnitude")
            plt.xlim(limits['time'])
            plt.ylim(limits['kh'])
            plt.grid(True)
            plt.tight_layout()
            plt.savefig(os.path.join(save_dir, f"{prefix}_{traj}_Kh.pdf"))
            plt.close()

            # --- Plot 2: Input Comparison ---
            plt.figure(figsize=(12, 6))
            if 'u_h_mag' in traj_data.columns: plt.plot(traj_data['timestamp'], traj_data['u_h_mag'], color='blue', label="Human Input ($u_h$)")
            if 'u_a_mag' in traj_data.columns: plt.plot(traj_data['timestamp'], traj_data['u_a_mag'], color='red', label="Adaptive Input ($u_a$)")
            plt.title(f"Control Input Comparison ($u_h$ vs. $u_a$)\n{title_info} | Run: {traj}")
            plt.xlabel("Timestamp")
            plt.ylabel("Control Input Magnitude")
            plt.xlim(limits['time'])
            plt.ylim(limits['u_mag'])
            plt.grid(True)
            plt.legend(loc='upper right')
            plt.tight_layout()
            plt.savefig(os.path.join(save_dir, f"{prefix}_{traj}_inputs.pdf"))
            plt.close()

    # ----------------------------------------
    # AGGREGATED PLOTS
    # ----------------------------------------
    # --- Plot 1: Kh Evolution (all) ---
    plt.figure(figsize=(10, 6))
    for traj in trajectories:
        traj_data = df[df['file_stem'] == traj]
        if 'Kh_value' in traj_data.columns and not traj_data['Kh_value'].isna().all():
            plt.plot(traj_data['timestamp'], traj_data['Kh_value'], color='purple', alpha=0.3)
    plt.title(f"Human Control Parameter ($K_h$) Evolution\n{title_info}")
    plt.xlabel("Timestamp")
    plt.ylabel("Estimated $K_h$ Magnitude")
    plt.xlim(limits['time'])
    plt.ylim(limits['kh'])
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f"{prefix}_all_Kh.pdf"))
    plt.close()

    # --- Plot 2: Input Comparison (all) ---
    plt.figure(figsize=(12, 6))
    for idx, traj in enumerate(trajectories):
        traj_data = df[df['file_stem'] == traj]
        label_h = "Human Input ($u_h$)" if idx == 0 else ""
        label_a = "Adaptive Input ($u_a$)" if idx == 0 else ""
        
        if 'u_h_mag' in traj_data.columns: plt.plot(traj_data['timestamp'], traj_data['u_h_mag'], color='blue', alpha=0.3, label=label_h)
        if 'u_a_mag' in traj_data.columns: plt.plot(traj_data['timestamp'], traj_data['u_a_mag'], color='red', alpha=0.3, label=label_a)
    plt.title(f"Control Input Comparison ($u_h$ vs. $u_a$)\n{title_info}")
    plt.xlabel("Timestamp")
    plt.ylabel("Control Input Magnitude")
    plt.xlim(limits['time'])
    plt.ylim(limits['u_mag'])
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f"{prefix}_all_inputs.pdf"))
    plt.close()

# ==========================================
# 3. Main Execution Workflow
# ==========================================

def main(data_directory="data", output_directory="authority_plots"):
    csv_files = glob.glob(os.path.join(data_directory, "**", "*.csv"), recursive=True)
    
    if not csv_files:
        print("No CSV files found in the specified directory.")
        return

    all_data = []
    for file in csv_files:
        df = pd.read_csv(file)
        if 'study_controller_mode' in df.columns: df['study_controller_mode'] = df['study_controller_mode'].astype(str).str.strip().str.lower()
        if 'study_phase' in df.columns: df['study_phase'] = df['study_phase'].astype(str).str.strip().str.lower()
            
        df = parse_and_calculate_inputs(df)
        df['file_stem'] = Path(file).stem
        all_data.append(df)
        
    master_df = pd.concat(all_data, ignore_index=True)
    
    limits = {
        'time': get_padded_limits([master_df['timestamp']], pad=0),
        'kh': get_padded_limits([master_df['Kh_value']]),
        'u_mag': get_padded_limits([master_df['u_h_mag'], master_df['u_a_mag']])
    }

    controllers = master_df['study_controller_mode'].dropna().unique()
    behaviors = master_df['study_phase'].dropna().unique()

    for controller in controllers:
        controller_df = master_df[master_df['study_controller_mode'] == controller]
        
        # 1. Plot aggregated all phases for this controller
        print(f"Generating aggregated all phases plots for {controller.upper()} Controller...")
        generate_authority_plots(controller_df, controller, "all_phases", output_directory, limits, aggregate_only=True)

        # 2. Iterate through specific phases
        for behavior in behaviors:
            behavior_df = controller_df[controller_df['study_phase'] == behavior]
            
            if not behavior_df.empty:
                print(f"Generating scaled & aggregated plots for {controller} controller - {behavior} phase...")
                generate_authority_plots(behavior_df, controller, behavior, output_directory, limits, aggregate_only=False)

if __name__ == "__main__":
    main(data_directory="../processed_logs", output_directory="../plots/authority_plots")