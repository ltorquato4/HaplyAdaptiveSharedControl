import os
import glob
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 1. JSON Parsing Logic
# ==========================================

def parse_mpc_json(df):
    """
    Parses the JSON strings in the 'K_a' column to extract weights and matrix diagonals.
    Assumes Q, R, and P are stored as 2D lists/arrays in the JSON.
    """
    w_comfort, w_trajectory, w_goal = [], [], []
    q_diag_0, r_diag_0, p_diag_0 = [], [], []
    
    for json_str in df['K_a']:
        try:
            # Parse the JSON string
            data = json.loads(json_str)
            
            # Extract scalar heuristic weights
            w_comfort.append(data.get('weight_comfort', np.nan))
            w_trajectory.append(data.get('weight_trajectory', np.nan))
            w_goal.append(data.get('weight_goal', np.nan))
            
            # Extract matrices and grab the first diagonal element
            Q = data.get('Q', [])
            R = data.get('R', [])
            P = data.get('P', [])
            
            q_diag_0.append(Q[0][0] if len(Q) > 0 and len(Q[0]) > 0 else np.nan)
            r_diag_0.append(R[0][0] if len(R) > 0 and len(R[0]) > 0 else np.nan)
            p_diag_0.append(P[0][0] if len(P) > 0 and len(P[0]) > 0 else np.nan)
            
        except (json.JSONDecodeError, TypeError, IndexError):
            # Handle empty rows or malformed JSON
            w_comfort.append(np.nan)
            w_trajectory.append(np.nan)
            w_goal.append(np.nan)
            q_diag_0.append(np.nan)
            r_diag_0.append(np.nan)
            p_diag_0.append(np.nan)

    # Assign parsed data back to the dataframe
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

def plot_heuristic_weights(trajectories, title_suffix, filename, output_dir):
    """
    Plots the evolution of heuristic weights over time.
    trajectories: list of DataFrames.
    """
    plt.figure(figsize=(10, 6))
    
    for idx, df in enumerate(trajectories):
        # Use solid lines if there's only one trajectory, otherwise make them semi-transparent
        alpha_val = 1.0 if len(trajectories) == 1 else 0.3
        
        # Only add labels for the first trajectory to avoid legend clutter
        label_c = 'Comfort' if idx == 0 else ""
        label_t = 'Trajectory' if idx == 0 else ""
        label_g = 'Goal' if idx == 0 else ""
        
        plt.plot(df['timestamp'], df['weight_comfort'], color='blue', alpha=alpha_val, label=label_c)
        plt.plot(df['timestamp'], df['weight_trajectory'], color='green', alpha=alpha_val, label=label_t)
        plt.plot(df['timestamp'], df['weight_goal'], color='red', alpha=alpha_val, label=label_g)

    plt.title(f"Heuristic Weighting Evolution\n({title_suffix})")
    plt.xlabel("Timestamp")
    plt.ylabel("Weight Value")
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename))
    plt.close()

def plot_cost_matrices(trajectories, title_suffix, filename, output_dir):
    """
    Plots the diagonal values of Q, R, and P matrices over time.
    """
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    
    for df in trajectories:
        alpha_val = 1.0 if len(trajectories) == 1 else 0.3
        
        axes[0].plot(df['timestamp'], df['Q_diag_0'], color='purple', alpha=alpha_val)
        axes[1].plot(df['timestamp'], df['R_diag_0'], color='orange', alpha=alpha_val)
        axes[2].plot(df['timestamp'], df['P_diag_0'], color='teal', alpha=alpha_val)

    axes[0].set_title(f"Q Matrix (State Cost) Diagonal\n({title_suffix})")
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
    plt.savefig(os.path.join(output_dir, filename))
    plt.close()

# ==========================================
# 3. Main Execution Workflow
# ==========================================

def main(data_directory="data", output_directory="mpc_plots"):
    os.makedirs(output_directory, exist_ok=True)
    csv_files = glob.glob(os.path.join(data_directory, "**", "*.csv"), recursive=True)
    
    adaptive_trajectories = []
    unique_phases = set()
    
    # Load and filter files
    for file in csv_files:
        df = pd.read_csv(file)
        
        # Proceed ONLY if this trajectory used the adaptive controller
        if 'study_controller_mode' in df.columns and df['study_controller_mode'].iloc[0].lower() == 'adaptive':
            # Track the phase for categorization later
            if 'study_phase' in df.columns:
                unique_phases.add(df['study_phase'].iloc[0].lower())
                
            # Parse the JSON string in the K_a column
            df = parse_mpc_json(df)
            adaptive_trajectories.append(df)
            
    if not adaptive_trajectories:
        print("No Adaptive controller trajectories found.")
        return

    print(f"Processed {len(adaptive_trajectories)} total adaptive trajectories.")

    # --- 1. All Phases Aggregated ---
    print("Generating aggregate plots across ALL study phases...")
    plot_heuristic_weights(adaptive_trajectories, "All Study Phases", "adaptive_all_phases_weights.pdf", output_directory)
    plot_cost_matrices(adaptive_trajectories, "All Study Phases", "adaptive_all_phases_matrices.pdf", output_directory)

    # --- 2. Separated by Study Phase ---
    for phase in unique_phases:
        print(f"Generating plots for study phase: '{phase}'...")
        
        # Filter the trajectories list for only data frames matching this phase
        phase_trajectories = [
            df for df in adaptive_trajectories 
            if 'study_phase' in df.columns and df['study_phase'].iloc[0].lower() == phase
        ]
        
        # Generate mode-specific plots
        plot_heuristic_weights(
            phase_trajectories, 
            f"Phase: {phase.title()}", 
            f"adaptive_{phase}_weights.pdf", 
            output_directory
        )
        plot_cost_matrices(
            phase_trajectories, 
            f"Phase: {phase.title()}", 
            f"adaptive_{phase}_matrices.pdf", 
            output_directory
        )

    print(f"Done. Plots saved to {output_directory}/")

if __name__ == "__main__":
    main(data_directory="../processed_logs", output_directory="../plots/mpc_plots")