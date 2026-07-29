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
# 2. Plotting & Phase Marker Logic
# ==========================================

def add_global_phase_labels(axes, df):
    if not isinstance(axes, (list, np.ndarray, tuple)):
        axes = [axes]
    bottom_ax = axes[-1]
    
    if 'study_phase' not in df.columns:
        return 1
        
    blocks = []
    
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
    
    blocks_df = pd.DataFrame(blocks).sort_values('min')
    
    merged_blocks = []
    for _, row in blocks_df.iterrows():
        if not merged_blocks:
            merged_blocks.append(row.to_dict())
        else:
            last = merged_blocks[-1]
            if row['phase'] == last['phase'] and row['min'] <= last['max'] + 60.0:
                last['max'] = max(last['max'], row['max'])
            else:
                merged_blocks.append(row.to_dict())
                
    trans = bottom_ax.get_xaxis_transform()
    levels = []
    
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
        
        y_arrow = -0.15 - (level * 0.12)
        y_text = -0.21 - (level * 0.12)
        
        for ax in axes:
            if p_start > df['timestamp'].min():
                ax.axvline(x=p_start, color='black', linestyle='--', linewidth=1.2, alpha=0.6)
            if p_end < df['timestamp'].max():
                ax.axvline(x=p_end, color='black', linestyle='--', linewidth=1.2, alpha=0.6)
                
            bottom_ax.annotate('', xy=(p_start, y_arrow), xytext=(p_end, y_arrow),
                    xycoords=trans, textcoords=trans,
                    arrowprops=dict(arrowstyle='<|-|>', color='black', shrinkA=0, shrinkB=0),
                    annotation_clip=False)
                    
            bottom_ax.text(p_mid, y_text, phase_name, transform=trans,
                ha='center', va='top', fontsize=11, color='black', clip_on=False)
                
    return len(levels)

def generate_controller_summary_plots(df, controller, behaviors, output_dir, limits):
    save_dir = os.path.join(output_dir, controller)
    os.makedirs(save_dir, exist_ok=True)
    
    common_norm_dist = np.linspace(0, 1, 500)
    
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
            beh_color = phase_config[behavior] # Fetch the specific color
            beh_df = df[df['study_phase'] == behavior]
            
            trajectories = beh_df['file_stem'].unique()
            max_end_x = 0
            interp_nx = []
            interp_ny = []
            
            for idx, traj in enumerate(trajectories):
                traj_data = beh_df[beh_df['file_stem'] == traj]
                ax.plot(traj_data['norm_x'], traj_data['norm_y'], alpha=0.2, color=beh_color, linestyle='--')
                
                end_x = traj_data['norm_end_x'].iloc[0]
                max_end_x = max(max_end_x, end_x)
                
                l_start = 'Start' if idx == 0 else ""
                l_end = 'End' if idx == 0 else ""
                ax.scatter(0, 0, c='green', marker='o', s=50, zorder=5, label=l_start, alpha=0.7)
                ax.scatter(end_x, 0, c='red', marker='X', s=50, zorder=5, label=l_end, alpha=0.7)
                
                traj_data_sorted = traj_data.dropna(subset=['normalized_distance', 'norm_x', 'norm_y']).sort_values(by='normalized_distance').drop_duplicates(subset=['normalized_distance'])
                if len(traj_data_sorted) > 1:
                    interp_nx.append(np.interp(common_norm_dist, traj_data_sorted['normalized_distance'], traj_data_sorted['norm_x']))
                    interp_ny.append(np.interp(common_norm_dist, traj_data_sorted['normalized_distance'], traj_data_sorted['norm_y']))
            
            ax.plot([0, max_end_x], [0, 0], 'k--', alpha=0.8, label='Reference', zorder=3)
            
            if interp_nx and interp_ny:
                mean_nx = np.mean(interp_nx, axis=0)
                mean_ny = np.mean(interp_ny, axis=0)
                std_ny = np.std(interp_ny, axis=0)
                
                ax.fill_between(mean_nx, mean_ny - std_ny, mean_ny + std_ny, color=beh_color, alpha=0.2, zorder=4, label="Variance")
                ax.plot(mean_nx, mean_ny, color=beh_color, linewidth=2, linestyle='--', label="Mean", zorder=10)
            
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