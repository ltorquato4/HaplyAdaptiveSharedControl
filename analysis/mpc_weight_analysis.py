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

# ==========================================
# 2. Plotting Functions
# ==========================================

def plot_heuristic_weights(trajectories, title_suffix, prefix, output_dir):
    for df in trajectories:
        file_stem = df['file_stem'].iloc[0]
        plt.figure(figsize=(10, 6))
        
        plt.plot(df['timestamp'], df['weight_comfort'], color='blue', label='Comfort')
        plt.plot(df['timestamp'], df['weight_trajectory'], color='green', label='Trajectory')
        plt.plot(df['timestamp'], df['weight_goal'], color='red', label='Goal')

        plt.title(f"Heuristic Weighting Evolution\n({title_suffix}) | Run: {file_stem}")
        plt.xlabel("Timestamp")
        plt.ylabel("Weight Value")
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f"{prefix}_{file_stem}_weights.pdf"))
        plt.close()

def plot_cost_matrices(trajectories, title_suffix, prefix, output_dir):
    for df in trajectories:
        file_stem = df['file_stem'].iloc[0]
        fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
        
        axes[0].plot(df['timestamp'], df['Q_diag_0'], color='purple')
        axes[1].plot(df['timestamp'], df['R_diag_0'], color='orange')
        axes[2].plot(df['timestamp'], df['P_diag_0'], color='teal')

        axes[0].set_title(f"Q Matrix (State Cost) Diagonal\n({title_suffix}) | Run: {file_stem}")
        axes[0].set_ylabel("Q Value")
        axes[0].grid(True)
        
        axes[1].set_title("R Matrix (Control Effort) Diagonal")
        axes[1].set_ylabel("R Value")
        axes[1].grid(True)
        
        axes[2].set_title("P Matrix (Terminal Cost) Diagonal")
        axes[2].set_xlabel("Timestamp")
        axes[2].set_ylabel("P Value")
        axes[2].grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f"{prefix}_{file_stem}_matrices.pdf"))
        plt.close()

# ==========================================
# 3. Main Execution Workflow
# ==========================================

def main(data_directory="data", output_directory="mpc_plots"):
    os.makedirs(output_directory, exist_ok=True)
    csv_files = glob.glob(os.path.join(data_directory, "**", "*.csv"), recursive=True)
    
    adaptive_trajectories = []
    unique_phases = set()
    
    for file in csv_files:
        df = pd.read_csv(file)
        if 'study_controller_mode' in df.columns and df['study_controller_mode'].iloc[0].lower() == 'adaptive':
            if 'study_phase' in df.columns:
                unique_phases.add(df['study_phase'].iloc[0].lower())
                
            df = parse_mpc_json(df)
            df['file_stem'] = Path(file).stem # Capture unique filename
            adaptive_trajectories.append(df)
            
    if not adaptive_trajectories:
        print("No Adaptive controller trajectories found.")
        return

    print(f"Processed {len(adaptive_trajectories)} total adaptive trajectories.")

    for phase in unique_phases:
        print(f"Generating plots for study phase: '{phase}'...")
        
        phase_trajectories = [
            df for df in adaptive_trajectories 
            if 'study_phase' in df.columns and df['study_phase'].iloc[0].lower() == phase
        ]
        
        plot_heuristic_weights(phase_trajectories, f"Phase: {phase.title()}", f"adaptive_{phase}", output_directory)
        plot_cost_matrices(phase_trajectories, f"Phase: {phase.title()}", f"adaptive_{phase}", output_directory)

    print(f"Done. Individual plots saved to {output_directory}/")

if __name__ == "__main__":
    main(data_directory="../processed_logs", output_directory="../plots/mpc_plots")