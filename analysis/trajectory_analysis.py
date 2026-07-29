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

def generate_plots(df, controller, behavior, output_dir, limits, aggregate_only=False):
    save_dir = os.path.join(output_dir, controller, behavior)
    os.makedirs(save_dir, exist_ok=True)
    
    trajectories = df['file_stem'].unique()
    title_phase = behavior.replace('_', ' ').title()

    # ----------------------------------------
    # INDIVIDUAL PLOTS
    # ----------------------------------------
    if not aggregate_only:
        for traj in trajectories:
            traj_data = df[df['file_stem'] == traj]
            prefix = f"{controller}_{behavior}_{traj}"
            
            # 1. 2D Cursor Trajectory
            plt.figure(figsize=(8, 6))
            plt.plot(traj_data['cursor_x'], traj_data['cursor_y'])
            plt.scatter(traj_data['start_x'].iloc[0], traj_data['start_y'].iloc[0], c='green', marker='o', s=100, label='Start', zorder=5)
            plt.scatter(traj_data['end_x'].iloc[0], traj_data['end_y'].iloc[0], c='red', marker='X', s=100, label='End', zorder=5)
            plt.title(f"2D Cursor Trajectory\nController: {controller.title()} | Phase: {title_phase} | Run: {traj}")
            plt.xlabel("Cursor X")
            plt.ylabel("Cursor Y")
            plt.xlim(limits['x_2d'])
            plt.ylim(limits['y_2d'])
            plt.legend()
            plt.grid(True)
            plt.savefig(os.path.join(save_dir, f"{prefix}_2d_trajectory.pdf"))
            plt.close()

            # 2. Origin-Aligned (f(x)=0) Trajectory
            plt.figure(figsize=(8, 6))
            plt.plot(traj_data['norm_x'], traj_data['norm_y'])
            plt.scatter(0, 0, c='green', marker='o', s=100, label='Start (0,0)', zorder=5)
            end_x = traj_data['norm_end_x'].iloc[0]
            plt.scatter(end_x, 0, c='red', marker='X', s=100, label='End (Aligned)', zorder=5)
            
            plt.plot([0, end_x], [0, 0], 'k--', alpha=0.5, label='f(x) = 0')
            
            plt.title(f"Aligned Trajectory (f(x)=0)\nController: {controller.title()} | Phase: {title_phase} | Run: {traj}")
            plt.xlabel("Normalized X")
            plt.ylabel("Normalized Y")
            plt.xlim(limits['norm_x'])
            plt.ylim(limits['norm_y'])
            plt.gca().set_aspect('equal', adjustable='box') 
            plt.legend()
            plt.grid(True)
            plt.savefig(os.path.join(save_dir, f"{prefix}_fx_aligned_trajectory.pdf"))
            plt.close()

            # 3. Positional Error
            plt.figure(figsize=(8, 6))
            traj_data_sorted = traj_data.sort_values(by='normalized_distance')
            plt.plot(traj_data_sorted['normalized_distance'], traj_data_sorted['orthogonal_error'])
            plt.title(f"Positional Error vs Normalized Distance\nController: {controller.title()} | Phase: {title_phase} | Run: {traj}")
            plt.xlabel("Position along Reference Trajectory (Normalized)")
            plt.ylabel("Orthogonal Error")
            plt.xlim(0, 1)
            plt.ylim(limits['error'])
            plt.grid(True)
            plt.savefig(os.path.join(save_dir, f"{prefix}_positional_error.pdf"))
            plt.close()

            # 4. Velocity Profiles
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
            ax1.plot(traj_data['timestamp'], traj_data['haply_vel_x'])
            ax2.plot(traj_data['timestamp'], traj_data['haply_vel_y'])

            ax1.set_title(f"Velocity Profile\nController: {controller.title()} | Phase: {title_phase} | Run: {traj}")
            ax1.set_ylabel("Velocity X")
            ax1.set_xlim(limits['time'])
            ax1.set_ylim(limits['vel_x'])
            ax1.grid(True)
            
            ax2.set_ylabel("Velocity Y")
            ax2.set_ylim(limits['vel_y'])
            ax2.grid(True)
            
            num_levels = add_global_phase_labels([ax1, ax2], traj_data)
            ax2.set_xlabel("Timestamp", labelpad=35 + (num_levels * 18))
            
            plt.tight_layout()
            plt.savefig(os.path.join(save_dir, f"{prefix}_velocity_profiles.pdf"), bbox_inches='tight')
            plt.close()

    # ----------------------------------------
    # AGGREGATED PLOTS
    # ----------------------------------------
    prefix_all = f"{controller}_{behavior}_all"
    common_norm_dist = np.linspace(0, 1, 500)
    
    # 1. 2D Cursor Trajectory (all)
    plt.figure(figsize=(8, 6))
    interp_cx = []
    interp_cy = []
    
    for idx, traj in enumerate(trajectories):
        traj_data = df[df['file_stem'] == traj]
        plt.plot(traj_data['cursor_x'], traj_data['cursor_y'], alpha=0.3)
        
        l_start = 'Start' if idx == 0 else ""
        l_end = 'End' if idx == 0 else ""
        plt.scatter(traj_data['start_x'].iloc[0], traj_data['start_y'].iloc[0], c='green', marker='o', s=100, label=l_start, zorder=5, alpha=0.7)
        plt.scatter(traj_data['end_x'].iloc[0], traj_data['end_y'].iloc[0], c='red', marker='X', s=100, label=l_end, zorder=5, alpha=0.7)
        
        traj_data_sorted = traj_data.dropna(subset=['normalized_distance', 'cursor_x', 'cursor_y']).sort_values(by='normalized_distance').drop_duplicates(subset=['normalized_distance'])
        if len(traj_data_sorted) > 1:
            interp_cx.append(np.interp(common_norm_dist, traj_data_sorted['normalized_distance'], traj_data_sorted['cursor_x']))
            interp_cy.append(np.interp(common_norm_dist, traj_data_sorted['normalized_distance'], traj_data_sorted['cursor_y']))

    if interp_cx and interp_cy:
        mean_cx = np.mean(interp_cx, axis=0)
        mean_cy = np.mean(interp_cy, axis=0)
        plt.plot(mean_cx, mean_cy, color='black', linewidth=3, linestyle='--', label='Mean Trajectory', zorder=10)
        
    plt.title(f"2D Cursor Trajectory\nController: {controller.title()} | {title_phase}")
    plt.xlabel("Cursor X")
    plt.ylabel("Cursor Y")
    plt.xlim(limits['x_2d'])
    plt.ylim(limits['y_2d'])
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(save_dir, f"{prefix_all}_2d_trajectory.pdf"))
    plt.close()

    # 2. Origin-Aligned (f(x)=0) Trajectory (all)
    plt.figure(figsize=(8, 6))
    max_end_x = 0
    interp_nx = []
    interp_ny = []
    
    for idx, traj in enumerate(trajectories):
        traj_data = df[df['file_stem'] == traj]
        plt.plot(traj_data['norm_x'], traj_data['norm_y'], alpha=0.3)
        
        l_start = 'Start (0,0)' if idx == 0 else ""
        l_end = 'End (Aligned)' if idx == 0 else ""
        
        end_x = traj_data['norm_end_x'].iloc[0]
        plt.scatter(0, 0, c='green', marker='o', s=100, label=l_start, zorder=5, alpha=0.7)
        plt.scatter(end_x, 0, c='red', marker='X', s=100, label=l_end, zorder=5, alpha=0.7)
        max_end_x = max(max_end_x, end_x)
        
        traj_data_sorted = traj_data.dropna(subset=['normalized_distance', 'norm_x', 'norm_y']).sort_values(by='normalized_distance').drop_duplicates(subset=['normalized_distance'])
        if len(traj_data_sorted) > 1:
            interp_nx.append(np.interp(common_norm_dist, traj_data_sorted['normalized_distance'], traj_data_sorted['norm_x']))
            interp_ny.append(np.interp(common_norm_dist, traj_data_sorted['normalized_distance'], traj_data_sorted['norm_y']))
            
    if interp_nx and interp_ny:
        mean_nx = np.mean(interp_nx, axis=0)
        mean_ny = np.mean(interp_ny, axis=0)
        plt.plot(mean_nx, mean_ny, color='black', linewidth=3, linestyle='--', label='Mean Trajectory', zorder=10)
        
    plt.plot([0, max_end_x], [0, 0], 'k--', alpha=0.5, label='f(x) = 0' if len(trajectories) > 0 else "") 
    plt.title(f"Aligned Trajectory (f(x)=0)\nController: {controller.title()} | {title_phase}")
    plt.xlabel("Normalized X")
    plt.ylabel("Normalized Y")
    plt.xlim(limits['norm_x'])
    plt.ylim(limits['norm_y'])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(save_dir, f"{prefix_all}_fx_aligned_trajectory.pdf"))
    plt.close()

    # 3. Positional Error (all)
    plt.figure(figsize=(8, 6))
    interpolated_errors = []
    for traj in trajectories:
        traj_data = df[df['file_stem'] == traj]
        traj_data_sorted = traj_data.dropna(subset=['normalized_distance', 'orthogonal_error']).sort_values(by='normalized_distance').drop_duplicates(subset=['normalized_distance'])
        
        plt.plot(traj_data_sorted['normalized_distance'], traj_data_sorted['orthogonal_error'], alpha=0.3)
        
        if len(traj_data_sorted) > 1:
            interp_error = np.interp(
                common_norm_dist, 
                traj_data_sorted['normalized_distance'], 
                traj_data_sorted['orthogonal_error']
            )
            interpolated_errors.append(interp_error)

    if interpolated_errors:
        mean_error = np.mean(interpolated_errors, axis=0)
        plt.plot(common_norm_dist, mean_error, color='black', linewidth=3, linestyle='--', label='Mean Error', zorder=10)
        plt.legend()

    plt.title(f"Positional Error vs Normalized Distance\nController: {controller.title()} | {title_phase}")
    plt.xlabel("Position along Reference Trajectory (Normalized)")
    plt.ylabel("Orthogonal Error")
    plt.xlim(0, 1)
    plt.ylim(limits['error'])
    plt.grid(True)
    plt.savefig(os.path.join(save_dir, f"{prefix_all}_positional_error.pdf"))
    plt.close()

    # 4. Velocity Profiles (all)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for traj in trajectories:
        traj_data = df[df['file_stem'] == traj]
        ax1.plot(traj_data['timestamp'], traj_data['haply_vel_x'])
        ax2.plot(traj_data['timestamp'], traj_data['haply_vel_y'])
        
    ax1.set_title(f"Velocity Profile\nController: {controller.title()} | {title_phase}")
    ax1.set_ylabel("Velocity X")
    ax1.set_xlim(limits['time'])
    ax1.set_ylim(limits['vel_x'])
    ax1.grid(True)
    
    ax2.set_ylabel("Velocity Y")
    ax2.set_ylim(limits['vel_y'])
    ax2.grid(True)
    
    num_levels = add_global_phase_labels([ax1, ax2], df)
    ax2.set_xlabel("Timestamp", labelpad=35 + (num_levels * 18))
    
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f"{prefix_all}_velocity_profiles.pdf"), bbox_inches='tight')
    plt.close()

def generate_controller_summary_plots(df, controller, behaviors, output_dir, limits):
    save_dir = os.path.join(output_dir, controller)
    os.makedirs(save_dir, exist_ok=True)
    
    common_norm_dist = np.linspace(0, 1, 500)
    colors = ['tab:blue', 'tab:orange', 'tab:green']
    
    # ==========================================
    # Plot 1: Aligned Trajectories (1x3 Grid)
    # ==========================================
    fig_traj, axes_traj = plt.subplots(1, 3, figsize=(18, 5), sharex=True, sharey=True)
    if not isinstance(axes_traj, np.ndarray):
        axes_traj = [axes_traj]
        
    min_norm_y = limits['norm_y'][0]
    max_norm_y = limits['norm_y'][1]
    
    for i in range(3):
        ax = axes_traj[i]
        if i < len(behaviors):
            behavior = behaviors[i]
            beh_df = df[df['study_phase'] == behavior]
            
            trajectories = beh_df['file_stem'].unique()
            max_end_x = 0
            interp_nx = []
            interp_ny = []
            
            for idx, traj in enumerate(trajectories):
                traj_data = beh_df[beh_df['file_stem'] == traj]
                ax.plot(traj_data['norm_x'], traj_data['norm_y'], alpha=0.2, color=colors[i])
                
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
            
            ax.plot([0, max_end_x], [0, 0], 'k--', alpha=0.8, label='f(x) = 0')
            
            if interp_nx and interp_ny:
                mean_nx = np.mean(interp_nx, axis=0)
                mean_ny = np.mean(interp_ny, axis=0)
                ax.plot(mean_nx, mean_ny, color=colors[i], linewidth=2, linestyle='--', label=f"Mean Path", zorder=10)
                
                min_norm_y = min(min_norm_y, np.min(mean_ny))
                max_norm_y = max(max_norm_y, np.max(mean_ny))
            
            ax.set_title(f"Aligned Trajectories\nPhase: {behavior.replace('_', ' ').title()}")
            ax.set_xlabel("Normalized X")
            if i == 0:
                ax.set_ylabel("Normalized Y")
            ax.grid(True)
            ax.set_aspect('equal', adjustable='box')
            ax.legend(loc='best', fontsize=9)
        else:
            ax.set_visible(False)
            
    pad_norm_y = (max_norm_y - min_norm_y) * 0.05 if max_norm_y != min_norm_y else 1.0
    axes_traj[0].set_ylim(min_norm_y - pad_norm_y, max_norm_y + pad_norm_y)
    axes_traj[0].set_xlim(limits['norm_x'])
    
    plt.tight_layout()
    fig_traj.savefig(os.path.join(save_dir, f"{controller}_summary_aligned_trajectories.pdf"), bbox_inches='tight')
    plt.close(fig_traj)
    
    # ==========================================
    # Plot 2: Mean Positional Error (4x1 Grid)
    # ==========================================
    fig_err, axes_err = plt.subplots(4, 1, figsize=(8, 10), sharex=True, sharey=True)
    if not isinstance(axes_err, np.ndarray):
        axes_err = [axes_err]
        
    fig_err.suptitle(f"Mean Positional Error\nController: {controller.title()}", fontsize=14)
    
    min_err_y = float('inf')
    max_err_y = float('-inf')
    
    all_interp_errors = []
    
    for i in range(3):
        ax = axes_err[i]
        ax.plot([0, 1], [0, 0], 'k--', alpha=0.5, linewidth=1.5, zorder=1)
        ax.grid(True)
        ax.set_ylabel("Error")
        
        if i < len(behaviors):
            behavior = behaviors[i]
            beh_df = df[df['study_phase'] == behavior]
            
            interp_errors = []
            for traj in beh_df['file_stem'].unique():
                traj_data = beh_df[beh_df['file_stem'] == traj].dropna(subset=['normalized_distance', 'orthogonal_error']).sort_values(by='normalized_distance').drop_duplicates(subset=['normalized_distance'])
                if len(traj_data) > 1:
                    err = np.interp(common_norm_dist, traj_data['normalized_distance'], traj_data['orthogonal_error'])
                    interp_errors.append(err)
                    all_interp_errors.append(err)
            
            if interp_errors:
                mean_err = np.mean(interp_errors, axis=0)
                std_err = np.std(interp_errors, axis=0)
                
                ax.fill_between(common_norm_dist, mean_err - std_err, mean_err + std_err, color=colors[i], alpha=0.2, zorder=4)
                ax.plot(common_norm_dist, mean_err, color=colors[i], linewidth=2, zorder=5)
                
                min_err_y = min(min_err_y, np.min(mean_err - std_err))
                max_err_y = max(max_err_y, np.max(mean_err + std_err))
                
            ax.text(0.02, 0.85, f"{behavior.replace('_', ' ').title()}", transform=ax.transAxes, fontsize=11, fontweight='bold', va='top')
        else:
            ax.set_visible(False)
            
    ax_all = axes_err[3]
    ax_all.plot([0, 1], [0, 0], 'k--', alpha=0.5, linewidth=1.5, zorder=1)
    ax_all.grid(True)
    ax_all.set_ylabel("Error")
    ax_all.set_xlabel("Position along Reference Trajectory (Normalized)")
    
    if all_interp_errors:
        overall_mean = np.mean(all_interp_errors, axis=0)
        overall_std = np.std(all_interp_errors, axis=0)
        
        ax_all.fill_between(common_norm_dist, overall_mean - overall_std, overall_mean + overall_std, color='black', alpha=0.1, zorder=9)
        ax_all.plot(common_norm_dist, overall_mean, color='black', linewidth=2, linestyle='-', alpha=0.8, zorder=10)
        
        min_err_y = min(min_err_y, np.min(overall_mean - overall_std))
        max_err_y = max(max_err_y, np.max(overall_mean + overall_std))
        
    ax_all.text(0.02, 0.85, "All Phases", transform=ax_all.transAxes, fontsize=11, fontweight='bold', va='top')
    
    if min_err_y == float('inf'): 
        min_err_y, max_err_y = 0, 0.1
        
    pad_err_y = (max_err_y - min_err_y) * 0.05
    axes_err[0].set_ylim(min_err_y - pad_err_y, max_err_y + pad_err_y)
    axes_err[0].set_xlim(0, 1)
    
    plt.tight_layout()
    fig_err.subplots_adjust(top=0.92) 
    fig_err.savefig(os.path.join(save_dir, f"{controller}_summary_positional_error.pdf"), bbox_inches='tight')
    plt.close(fig_err)

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
    
    limits = {
        'x_2d': get_padded_limits([master_df['cursor_x'], master_df['start_x'], master_df['end_x']]),
        'y_2d': get_padded_limits([master_df['cursor_y'], master_df['start_y'], master_df['end_y']]),
        'norm_x': get_padded_limits([master_df['norm_x'], master_df['norm_end_x'], pd.Series([0])]),
        'norm_y': get_padded_limits([master_df['norm_y'], master_df['norm_end_y'], pd.Series([0])]),
        'time': get_padded_limits([master_df['timestamp']], pad=0),
        'error': get_padded_limits([master_df['orthogonal_error']]),
        'vel_x': get_padded_limits([master_df['haply_vel_x']]),
        'vel_y': get_padded_limits([master_df['haply_vel_y']])
    }
    controllers = master_df['study_controller_mode'].dropna().unique()
    behaviors = master_df['study_phase'].dropna().unique()

    for controller in controllers:
        controller_df = master_df[master_df['study_controller_mode'] == controller]
        
        print(f"Generating aggregated all phases plots for {controller} controller...")
        generate_plots(controller_df, controller, "all_phases", output_directory, limits, aggregate_only=True)

        for behavior in behaviors:
            behavior_df = controller_df[controller_df['study_phase'] == behavior]
            
            if not behavior_df.empty:
                print(f"Generating scaled & aggregated plots for {controller} controller - {behavior} phase...")
                generate_plots(behavior_df, controller, behavior, output_directory, limits, aggregate_only=False)

        print(f"Generating summary dashboard plots for {controller} controller...")
        generate_controller_summary_plots(controller_df, controller, behaviors, output_directory, limits)

if __name__ == "__main__":
    main(data_directory="../processed_logs", output_directory="../plots/trajectory_plots")