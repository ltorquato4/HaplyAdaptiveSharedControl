import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# ==========================================
# 1. Math & Metric Calculations
# ==========================================

def calculate_metrics(df):
    line_vec_x = df['end_x'] - df['start_x']
    line_vec_y = df['end_y'] - df['start_y']
    line_len_sq = line_vec_x**2 + line_vec_y**2
    line_len = np.sqrt(line_len_sq)

    point_vec_x = df['cursor_x'] - df['start_x']
    point_vec_y = df['cursor_y'] - df['start_y']

    # Absolute orthogonal error (magnitude of deviation)
    cross_prod = np.abs(point_vec_x * line_vec_y - point_vec_y * line_vec_x)
    df['orthogonal_error'] = np.where(line_len == 0, 0, cross_prod / line_len)

    # Position along the reference trajectory
    dot_prod = (point_vec_x * line_vec_x) + (point_vec_y * line_vec_y)
    df['normalized_distance'] = np.where(line_len_sq == 0, 0, dot_prod / line_len_sq)

    # --- f(x)=0 (Ursprungsgerade) Transformation ---
    theta = np.arctan2(line_vec_y, line_vec_x)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    
    # Apply 2D rotation matrix by -theta
    df['norm_x'] = point_vec_x * cos_theta + point_vec_y * sin_theta
    df['norm_y'] = -point_vec_x * sin_theta + point_vec_y * cos_theta
    
    # The end point is now perfectly on the X-axis
    df['norm_end_x'] = line_len
    df['norm_end_y'] = 0.0

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
# 2. Plotting Logic
# ==========================================

def generate_controller_summary_plots(df, controller, behaviors, output_dir, limits):
    save_dir = os.path.join(output_dir, controller)
    os.makedirs(save_dir, exist_ok=True)
    
    # Define 101 bin edges to create 100 discrete progress bins (0% to 100%)
    bin_edges = np.linspace(0.0, 1.0, 101)
    
    # Define explicit order and colors according to instructions
    phase_config = {
        'careful': 'tab:green',
        'normal': 'tab:orange',
        'aggressive': 'tab:red'
    }
    
    # Filter and order the behaviors present in the actual data
    ordered_behaviors = [b for b in phase_config.keys() if b in behaviors]
    
    # ==========================================
    # Plot 1: Aligned Trajectories (1x3 Grid)
    # ==========================================
    fig_traj, axes_traj = plt.subplots(1, 3, figsize=(18, 5), sharex=True, sharey=True)
    if not isinstance(axes_traj, np.ndarray):
        axes_traj = [axes_traj]
        
    for i in range(3):
        ax = axes_traj[i]
        if i < len(ordered_behaviors):
            behavior = ordered_behaviors[i]
            beh_color = phase_config[behavior] 
            beh_df = df[df['study_phase'] == behavior]
            
            trajectories = beh_df['file_stem'].unique()
            max_end_x = 0
            
            # Plot individual raw trajectories first
            for idx, traj in enumerate(trajectories):
                traj_data = beh_df[beh_df['file_stem'] == traj]
                ax.plot(traj_data['norm_x'], traj_data['norm_y'], alpha=0.5, color=beh_color, linestyle='--')
                
                end_x = traj_data['norm_end_x'].iloc[0]
                max_end_x = max(max_end_x, end_x)
                
                l_start = 'Start' if idx == 0 else ""
                l_end = 'End' if idx == 0 else ""
                ax.scatter(0, 0, c='green', marker='o', s=50, zorder=5, label=l_start, alpha=0.7)
                ax.scatter(end_x, 0, c='red', marker='X', s=50, zorder=5, label=l_end, alpha=0.7)
            
            ax.plot([0, max_end_x], [0, 0], 'k--', alpha=0.8, label='Reference', zorder=3)
            
            # Calculate grouped bin aggregations across all trajectories in this phase
            beh_df_clean = beh_df.dropna(subset=['normalized_distance', 'norm_x', 'norm_y']).copy()
            beh_df_clean = beh_df_clean[(beh_df_clean['normalized_distance'] >= 0.0) & (beh_df_clean['normalized_distance'] <= 1.0)]
            beh_df_clean['progress_bin'] = pd.cut(beh_df_clean['normalized_distance'], bin_edges, labels=False, include_lowest=True)
            
            summary = beh_df_clean.groupby('progress_bin')[['norm_x', 'norm_y']].agg(['mean', 'std'])
            
            if not summary.empty:
                mean_nx = summary[('norm_x', 'mean')].to_numpy()
                mean_ny = summary[('norm_y', 'mean')].to_numpy()
                std_ny = summary[('norm_y', 'std')].fillna(0).to_numpy() # Fill NaNs for bins with only 1 sample
                
                ax.fill_between(mean_nx, mean_ny - std_ny, mean_ny + std_ny, color=beh_color, alpha=0.2, zorder=4, label="Variance")
                ax.plot(mean_nx, mean_ny, color=beh_color, linewidth=2, label="Mean", zorder=10)
            
            ax.set_title(f"Phase: {behavior.replace('_', ' ').title()}")
            ax.set_xlabel("Normalized X")
            if i == 0:
                ax.set_ylabel("Normalized Y")
            ax.grid(True)
            ax.set_aspect('equal', adjustable='box')
            
            # Force Legend location to top right with an opaque background
            ax.legend(loc='upper right', fontsize=9, framealpha=0.95, edgecolor='gray')
        else:
            ax.set_visible(False)
            
    # Apply the locally calculated limits specifically for THIS controller mode
    y_min, y_max = limits['norm_y']
    
    # FIXED absolute padding added to the top to accommodate the legend safely
    fixed_top_padding = 0.035
    axes_traj[0].set_ylim(y_min, y_max + fixed_top_padding)
    axes_traj[0].set_xlim(limits['norm_x'])
    
    plt.tight_layout()
    fig_traj.savefig(os.path.join(save_dir, f"{controller}_summary_aligned_trajectories.pdf"), bbox_inches='tight')
    plt.close(fig_traj)

# ==========================================
# 3. Main Execution Workflow
# ==========================================

def main(data_directory="data", output_directory="analysis_plots"):
    csv_files = glob.glob(os.path.join(data_directory, "**", "*.csv"), recursive=True)
    all_data = []
    
    for file in csv_files:
        try:
            df = pd.read_csv(file)
        except pd.errors.EmptyDataError:
            print(f"Skipping empty file: {file}")
            continue
            
        # SAFETY CHECK: Ensure critical columns exist before processing
        required_cols = ['end_x', 'start_x', 'end_y', 'start_y', 'cursor_x', 'cursor_y']
        if not all(col in df.columns for col in required_cols):
            print(f"Skipping {Path(file).name}: Missing required coordinate columns.")
            continue
        
        if 'study_controller_mode' in df.columns: df['study_controller_mode'] = df['study_controller_mode'].astype(str).str.strip().str.lower()
        if 'study_phase' in df.columns: df['study_phase'] = df['study_phase'].astype(str).str.strip().str.lower()
            
        df['file_stem'] = Path(file).stem
        df = calculate_metrics(df)
        all_data.append(df)
        
    if not all_data:
        print("No valid trajectories found.")
        return

    master_df = pd.concat(all_data, ignore_index=True)
    
    controllers = master_df['study_controller_mode'].dropna().unique()
    behaviors = master_df['study_phase'].dropna().unique()

    for controller in controllers:
        controller_df = master_df[master_df['study_controller_mode'] == controller]
        
        # Calculate limits SPECIFICALLY for the subset of data belonging to this controller
        limits = {
            'x_2d': get_padded_limits([controller_df['cursor_x'], controller_df['start_x'], controller_df['end_x']]),
            'y_2d': get_padded_limits([controller_df['cursor_y'], controller_df['start_y'], controller_df['end_y']]),
            'norm_x': get_padded_limits([controller_df['norm_x'], controller_df['norm_end_x'], pd.Series([0])]),
            'norm_y': get_padded_limits([controller_df['norm_y'], controller_df['norm_end_y'], pd.Series([0])]),
            'time': get_padded_limits([controller_df['timestamp']], pad=0),
            'error': get_padded_limits([controller_df['orthogonal_error']]),
            'vel_x': get_padded_limits([controller_df['haply_vel_x']]),
            'vel_y': get_padded_limits([controller_df['haply_vel_y']])
        }
        
        print(f"Generating summary dashboard plots for {controller} controller...")
        generate_controller_summary_plots(controller_df, controller, behaviors, output_directory, limits)

if __name__ == "__main__":
    main(data_directory="../processed_logs", output_directory="../plots/trajectory_plots")