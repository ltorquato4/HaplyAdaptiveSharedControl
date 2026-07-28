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
            # Handle potential NaN values or empty strings in K_a
            if pd.isna(json_str) or not str(json_str).strip():
                raise ValueError("Empty JSON")
                
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
            
        except (json.JSONDecodeError, TypeError, IndexError, ValueError):
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
    
    # Forward-fill and backward-fill in case 'fixed' controllers only log the JSON once
    cols_to_fill = ['weight_comfort', 'weight_trajectory', 'weight_goal', 'Q_diag_0', 'R_diag_0', 'P_diag_0']
    df[cols_to_fill] = df[cols_to_fill].ffill().bfill()
    
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
# 2. Plotting & Phase Marker Logic
# ==========================================

def add_global_phase_labels(axes, df):
    """
    Finds the exact time intervals for all phases across ALL trajectories,
    merges them into chronological blocks, draws vertical dividers across all subplots,
    and draws dimension arrows for each execution mode on the bottom axis.
    """
    # Allow passing a single axis or an array/list of axes
    if not isinstance(axes, (list, np.ndarray)):
        axes = [axes]
    bottom_ax = axes[-1]
    
    if 'study_phase' not in df.columns:
        return 1
        
    blocks = []
    
    # 1. Identify continuous phase blocks within EVERY trajectory
    for traj in df['file_stem'].unique():
        traj_df = df[df['file_stem'] == traj].sort_values('timestamp')
        if traj_df.empty: 
            continue
        
        traj_df['block'] = (traj_df['study_phase'] != traj_df['study_phase'].shift(1)).cumsum()
        for _, block_df in traj_df.groupby('block'):
            blocks.append({
                'phase': block_df['study_phase'].iloc[0],
                'min': block_df['timestamp'].min(),
                'max': block_df['timestamp'].max()
            })
            
    if not blocks: 
        return 1
    
    # 2. Sort all extracted blocks chronologically
    blocks_df = pd.DataFrame(blocks).sort_values('min')
    
    # 3. Merge overlapping or sequential blocks of the SAME phase
    merged_blocks = []
    for _, row in blocks_df.iterrows():
        if not merged_blocks:
            merged_blocks.append(row.to_dict())
        else:
            last = merged_blocks[-1]
            if row['phase'] == last['phase'] and row['min'] <= last['max'] + 5.0: 
                last['max'] = max(last['max'], row['max'])
            else:
                merged_blocks.append(row.to_dict())
                
    trans = bottom_ax.get_xaxis_transform()
    levels = [] 
    
    # 4. Draw the boundaries, arrows, and labels
    for idx, row in pd.DataFrame(merged_blocks).iterrows():
        phase_name = str(row['phase']).replace('_', ' ').title()
        p_start = row['min']
        p_end = row['max']
        p_mid = (p_start + p_end) / 2
        
        level = 0
        for l_idx, l_end in enumerate(levels):
            if p_start >= l_end:
                level = l_idx
                break
        else:
            level = len(levels)
            levels.append(p_end)
        
        levels[level] = p_end
        
        # Scaling Y offsets slightly larger since the subplot height is smaller
        y_arrow = -0.12 - (level * 0.12)
        y_text = -0.18 - (level * 0.12)
        
        # Draw vertical dividers on ALL axes in the subplot figure
        for ax in axes:
            if p_start > df['timestamp'].min():
                ax.axvline(x=p_start, color='black', linestyle='--', linewidth=1.2, alpha=0.6)
            if p_end < df['timestamp'].max():
                ax.axvline(x=p_end, color='black', linestyle='--', linewidth=1.2, alpha=0.6)
                
        # Draw horizontal dimension arrow ONLY on the bottom axis
        bottom_ax.annotate('', xy=(p_start, y_arrow), xytext=(p_end, y_arrow),
                    xycoords=trans, textcoords=trans,
                    arrowprops=dict(arrowstyle='<|-|>', color='black', shrinkA=0, shrinkB=0),
                    annotation_clip=False)
                    
        # Place the execution mode label ONLY on the bottom axis
        bottom_ax.text(p_mid, y_text, phase_name, transform=trans,
                ha='center', va='top', fontsize=11, color='black', clip_on=False)
                
    return len(levels)

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
            fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
            axes[0].plot(traj_data['timestamp'], traj_data['weight_comfort'], color='blue')
            axes[1].plot(traj_data['timestamp'], traj_data['weight_trajectory'], color='green')
            axes[2].plot(traj_data['timestamp'], traj_data['weight_goal'], color='red')
            
            axes[0].set_title(f"Comfort Weight\n{title_info} | Run: {traj}")
            axes[0].set_ylabel("Weight Value")
            axes[0].set_xlim(limits['time'])
            axes[0].set_ylim(limits['weight_comfort'])
            axes[0].grid(True)

            axes[1].set_title("Trajectory Weight")
            axes[1].set_ylabel("Weight Value")
            axes[1].set_ylim(limits['weight_trajectory'])
            axes[1].grid(True)

            axes[2].set_title("Goal Weight")
            axes[2].set_ylabel("Weight Value")
            axes[2].set_ylim(limits['weight_goal'])
            axes[2].grid(True)

            # Draw phase markers and adjust labelpad dynamically
            num_levels = add_global_phase_labels(axes, traj_data)
            axes[2].set_xlabel("Timestamp", labelpad=35 + (num_levels * 18))

            plt.tight_layout()
            plt.savefig(os.path.join(save_dir, f"{prefix}_{traj}_weights.pdf"), bbox_inches='tight')
            plt.close()

            # --- Plot 2: Cost Matrices ---
            fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
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
            axes[2].set_ylabel("P Value")
            axes[2].set_ylim(limits['P'])
            axes[2].grid(True)

            num_levels = add_global_phase_labels(axes, traj_data)
            axes[2].set_xlabel("Timestamp", labelpad=35 + (num_levels * 18))

            plt.tight_layout()
            plt.savefig(os.path.join(save_dir, f"{prefix}_{traj}_matrices.pdf"), bbox_inches='tight')
            plt.close()

    # ----------------------------------------
    # AGGREGATED PLOTS
    # ----------------------------------------
    # --- Plot 1: Heuristic Weights (all) ---
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    for idx, traj in enumerate(trajectories):
        traj_data = df[df['file_stem'] == traj]
        axes[0].plot(traj_data['timestamp'], traj_data['weight_comfort'], color='blue')
        axes[1].plot(traj_data['timestamp'], traj_data['weight_trajectory'], color='green')
        axes[2].plot(traj_data['timestamp'], traj_data['weight_goal'], color='red')
            
    axes[0].set_title(f"Comfort Weight\n{title_info}")
    axes[0].set_ylabel("Weight Value")
    axes[0].set_xlim(limits['time'])
    axes[0].set_ylim(limits['weight_comfort'])
    axes[0].grid(True)

    axes[1].set_title("Trajectory Weight")
    axes[1].set_ylabel("Weight Value")
    axes[1].set_ylim(limits['weight_trajectory'])
    axes[1].grid(True)

    axes[2].set_title("Goal Weight")
    axes[2].set_ylabel("Weight Value")
    axes[2].set_ylim(limits['weight_goal'])
    axes[2].grid(True)

    num_levels = add_global_phase_labels(axes, df)
    axes[2].set_xlabel("Timestamp", labelpad=35 + (num_levels * 18))

    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f"{prefix}_all_weights.pdf"), bbox_inches='tight')
    plt.close()

    # --- Plot 2: Cost Matrices (all) ---
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    for traj in trajectories:
        traj_data = df[df['file_stem'] == traj]
        axes[0].plot(traj_data['timestamp'], traj_data['Q_diag_0'], color='purple')
        axes[1].plot(traj_data['timestamp'], traj_data['R_diag_0'], color='orange')
        axes[2].plot(traj_data['timestamp'], traj_data['P_diag_0'], color='teal')

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
    axes[2].set_ylabel("P Value")
    axes[2].set_ylim(limits['P'])
    axes[2].grid(True)
    
    num_levels = add_global_phase_labels(axes, df)
    axes[2].set_xlabel("Timestamp", labelpad=35 + (num_levels * 18))

    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f"{prefix}_all_matrices.pdf"), bbox_inches='tight')
    plt.close()

# ==========================================
# 3. Main Execution Workflow
# ==========================================

def main(data_directory="data", output_directory="mpc_plots"):
    csv_files = glob.glob(os.path.join(data_directory, "**", "*.csv"), recursive=True)
    all_data = []
    
    for file in csv_files:
        df = pd.read_csv(file)
        if 'study_controller_mode' in df.columns: 
            df['study_controller_mode'] = df['study_controller_mode'].astype(str).str.strip().str.lower()
        if 'study_phase' in df.columns: 
            df['study_phase'] = df['study_phase'].astype(str).str.strip().str.lower()
                
        df = parse_mpc_json(df)
        df['file_stem'] = Path(file).stem
        all_data.append(df)
            
    if not all_data:
        print("No valid trajectories found.")
        return

    master_df = pd.concat(all_data, ignore_index=True)
    
    # Calculate independent limits for each weight
    limits = {
        'time': get_padded_limits([master_df['timestamp']], pad=0),
        'weight_comfort': get_padded_limits([master_df['weight_comfort']]),
        'weight_trajectory': get_padded_limits([master_df['weight_trajectory']]),
        'weight_goal': get_padded_limits([master_df['weight_goal']]),
        'Q': get_padded_limits([master_df['Q_diag_0']]),
        'R': get_padded_limits([master_df['R_diag_0']]),
        'P': get_padded_limits([master_df['P_diag_0']])
    }

    # Automatically captures all controllers, including both 'fixed' and 'adaptive'
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
                print(f"Generating scaled & aggregated plots for {controller.upper()} controller - {behavior.upper()} phase...")
                generate_mpc_plots(behavior_df, controller, behavior, output_directory, limits, aggregate_only=False)

if __name__ == "__main__":
    main(data_directory="../processed_logs", output_directory="../plots/mpc_plots")