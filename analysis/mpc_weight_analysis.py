import os
import glob
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# ==========================================
# 1. JSON Parsing Logic
# ==========================================

def parse_mpc_json(df):
    w_comfort, w_trajectory, w_goal = [], [], []
    q_diag_0, r_diag_0, p_diag_0 = [], [], []
    
    if 'K_a' not in df.columns:
        df[['weight_comfort', 'weight_trajectory', 'weight_goal', 'Q_diag_0', 'R_diag_0', 'P_diag_0']] = np.nan
        return df

    for json_str in df['K_a']:
        try:
            data = json.loads(json_str)
            w_comfort.append(data.get('weight_comfort', np.nan))
            w_trajectory.append(data.get('weight_trajectory', np.nan))
            w_goal.append(data.get('weight_goal', np.nan))
            
            Q = data.get('Q', [])
            R = data.get('R', [])
            P = data.get('P', [])
            
            q_diag_0.append(Q[0][0] if len(Q) > 0 and len(Q[0]) > 0 else np.nan)
            r_diag_0.append(R[0][0] if len(R) > 0 and len(R[0]) > 0 else np.nan)
            p_diag_0.append(P[0][0] if len(P) > 0 and len(P[0]) > 0 else np.nan)
            
        except (json.JSONDecodeError, TypeError, IndexError):
            w_comfort.append(np.nan)
            w_trajectory.append(np.nan)
            w_goal.append(np.nan)
            q_diag_0.append(np.nan)
            r_diag_0.append(np.nan)
            p_diag_0.append(np.nan)

    df['weight_comfort'] = w_comfort
    df['weight_trajectory'] = w_trajectory
    df['weight_goal'] = w_goal
    df['Q_diag_0'] = q_diag_0
    df['R_diag_0'] = r_diag_0
    df['P_diag_0'] = p_diag_0
    
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

def generate_mpc_plots(df, controller, behavior, output_dir, limits, aggregate_only=False):
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
            
            # --- Plot 1: Heuristic Weights ---
            plt.figure(figsize=(10, 6))
            if not traj_data['weight_comfort'].isna().all():
                plt.plot(traj_data['timestamp'], traj_data['weight_comfort'], color='blue', label='Comfort')
                plt.plot(traj_data['timestamp'], traj_data['weight_trajectory'], color='green', label='Trajectory')
                plt.plot(traj_data['timestamp'], traj_data['weight_goal'], color='red', label='Goal')
            plt.title(f"Heuristic Weighting Evolution\n{title_info} | Run: {traj}")
            plt.xlabel("Timestamp")
            plt.ylabel("Weight Value")
            plt.xlim(limits['time'])
            plt.ylim(limits['weights'])
            plt.grid(True)
            plt.legend()
            plt.tight_layout()
            plt.savefig(os.path.join(save_dir, f"{prefix}_{traj}_weights.pdf"))
            plt.close()

            # --- Plot 2: Cost Matrices ---
            fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
            if not traj_data['Q_diag_0'].isna().all():
                axes[0].plot(traj_data['timestamp'], traj_data['Q_diag_0'], color='purple')
                axes[1].plot(traj_data['timestamp'], traj_data['R_diag_0'], color='orange')
                axes[2].plot(traj_data['timestamp'], traj_data['P_diag_0'], color='teal')

            axes[0].set_title(f"Q Matrix (State Cost) Diagonal\n{title_info} | Run: {traj}")
            axes[0].set_ylabel("Q Value")
            axes[0].set_xlim(limits['time'])
            axes[0].set_ylim(limits['Q'])
            axes[0].grid(True)
            
            axes[1].set_title("R Matrix (Control Effort) Diagonal")
            axes[1].set_ylabel("R Value")
            axes[1].set_ylim(limits['R'])
            axes[1].grid(True)
            
            axes[2].set_title("P Matrix (Terminal Cost) Diagonal")
            axes[2].set_xlabel("Timestamp")
            axes[2].set_ylabel("P Value")
            axes[2].set_ylim(limits['P'])
            axes[2].grid(True)
            plt.tight_layout()
            plt.savefig(os.path.join(save_dir, f"{prefix}_{traj}_matrices.pdf"))
            plt.close()

    # ----------------------------------------
    # AGGREGATED PLOTS
    # ----------------------------------------
    # --- Plot 1: Heuristic Weights (all) ---
    plt.figure(figsize=(10, 6))
    for idx, traj in enumerate(trajectories):
        traj_data = df[df['file_stem'] == traj]
        l_c = 'Comfort' if idx == 0 else ""
        l_t = 'Trajectory' if idx == 0 else ""
        l_g = 'Goal' if idx == 0 else ""
        
        if not traj_data['weight_comfort'].isna().all():
            plt.plot(traj_data['timestamp'], traj_data['weight_comfort'], color='blue', alpha=0.3, label=l_c)
            plt.plot(traj_data['timestamp'], traj_data['weight_trajectory'], color='green', alpha=0.3, label=l_t)
            plt.plot(traj_data['timestamp'], traj_data['weight_goal'], color='red', alpha=0.3, label=l_g)
    plt.title(f"Heuristic Weighting Evolution\n{title_info}")
    plt.xlabel("Timestamp")
    plt.ylabel("Weight Value")
    plt.xlim(limits['time'])
    plt.ylim(limits['weights'])
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f"{prefix}_all_weights.pdf"))
    plt.close()

    # --- Plot 2: Cost Matrices (all) ---
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    for traj in trajectories:
        traj_data = df[df['file_stem'] == traj]
        if not traj_data['Q_diag_0'].isna().all():
            axes[0].plot(traj_data['timestamp'], traj_data['Q_diag_0'], color='purple', alpha=0.3)
            axes[1].plot(traj_data['timestamp'], traj_data['R_diag_0'], color='orange', alpha=0.3)
            axes[2].plot(traj_data['timestamp'], traj_data['P_diag_0'], color='teal', alpha=0.3)

    axes[0].set_title(f"Q Matrix (State Cost) Diagonal\n{title_info}")
    axes[0].set_ylabel("Q Value")
    axes[0].set_xlim(limits['time'])
    axes[0].set_ylim(limits['Q'])
    axes[0].grid(True)
    
    axes[1].set_title("R Matrix (Control Effort) Diagonal")
    axes[1].set_ylabel("R Value")
    axes[1].set_ylim(limits['R'])
    axes[1].grid(True)
    
    axes[2].set_title("P Matrix (Terminal Cost) Diagonal")
    axes[2].set_xlabel("Timestamp")
    axes[2].set_ylabel("P Value")
    axes[2].set_ylim(limits['P'])
    axes[2].grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f"{prefix}_all_matrices.pdf"))
    plt.close()

# ==========================================
# 3. Main Execution Workflow
# ==========================================

def main(data_directory="data", output_directory="mpc_plots"):
    csv_files = glob.glob(os.path.join(data_directory, "**", "*.csv"), recursive=True)
    all_data = []
    
    for file in csv_files:
        df = pd.read_csv(file)
        if 'study_controller_mode' in df.columns: df['study_controller_mode'] = df['study_controller_mode'].astype(str).str.strip().str.lower()
        if 'study_phase' in df.columns: df['study_phase'] = df['study_phase'].astype(str).str.strip().str.lower()
                
        df = parse_mpc_json(df)
        df['file_stem'] = Path(file).stem
        all_data.append(df)
            
    if not all_data:
        print("No valid trajectories found.")
        return

    master_df = pd.concat(all_data, ignore_index=True)
    
    limits = {
        'time': get_padded_limits([master_df['timestamp']], pad=0),
        'weights': get_padded_limits([master_df['weight_comfort'], master_df['weight_trajectory'], master_df['weight_goal']]),
        'Q': get_padded_limits([master_df['Q_diag_0']]),
        'R': get_padded_limits([master_df['R_diag_0']]),
        'P': get_padded_limits([master_df['P_diag_0']])
    }

    controllers = master_df['study_controller_mode'].dropna().unique()
    behaviors = master_df['study_phase'].dropna().unique()

    for controller in controllers:
        controller_df = master_df[master_df['study_controller_mode'] == controller]
        
        # 1. Plot aggregated all phases for this controller
        print(f"Generating aggregated all phases plots for {controller.upper()} Controller...")
        generate_mpc_plots(controller_df, controller, "all_phases", output_directory, limits, aggregate_only=True)

        # 2. Iterate through specific phases
        for behavior in behaviors:
            behavior_df = controller_df[controller_df['study_phase'] == behavior]
            
            if not behavior_df.empty:
                print(f"Generating scaled & aggregated plots for {controller} controller - {behavior} phase...")
                generate_mpc_plots(behavior_df, controller, behavior, output_directory, limits, aggregate_only=False)

if __name__ == "__main__":
    main(data_directory="../processed_logs", output_directory="../plots/mpc_plots")