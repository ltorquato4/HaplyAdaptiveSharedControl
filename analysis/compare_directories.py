import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 1. Math & Metric Calculations
# ==========================================

def calculate_metrics(df):
    """Calculates orthogonal error, normalized distance, and f(x)=0 transformation."""
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

# ==========================================
# 2. Main Execution Workflow
# ==========================================

def plot_user_mean_trajectories(base_data_dir="../processed_logs", output_dir="../plots/comparison_plots"):
    """Plots the mean aligned trajectory of each user per controller mode and phase."""
    os.makedirs(output_dir, exist_ok=True)
    common_norm_dist = np.linspace(0, 1, 500)
    
    # Nested dict structure: {mode: {phase: {run_name: {'nx': [], 'ny': [], 'max_end_x': 0}}}}
    mode_data = {}
    
    subdirectories = [f.path for f in os.scandir(base_data_dir) if f.is_dir()]
    
    if not subdirectories:
        print(f"No subdirectories found in {base_data_dir}")
        return

    # Extract and calculate data for all runs
    for subdir in sorted(subdirectories):
        run_name = os.path.basename(subdir)
        csv_files = glob.glob(os.path.join(subdir, "**", "*.csv"), recursive=True)
        
        for file in csv_files:
            try:
                df = pd.read_csv(file)
                
                # SAFETY CHECK
                required_cols = ['end_x', 'start_x', 'end_y', 'start_y', 'cursor_x', 'cursor_y', 'study_controller_mode', 'study_phase']
                if not all(col in df.columns for col in required_cols):
                    continue
                
                # Standardize strings
                df['study_controller_mode'] = df['study_controller_mode'].astype(str).str.strip().str.lower()
                df['study_phase'] = df['study_phase'].astype(str).str.strip().str.lower()
                df = calculate_metrics(df)

                # Group by mode
                for mode, mode_df in df.groupby('study_controller_mode'):
                    if mode not in mode_data:
                        mode_data[mode] = {}
                        
                    # Store INDIVIDUAL PHASE data for this file
                    for phase, phase_df in mode_df.groupby('study_phase'):
                        if phase not in mode_data[mode]:
                            mode_data[mode][phase] = {}
                        if run_name not in mode_data[mode][phase]:
                            mode_data[mode][phase][run_name] = {'nx': [], 'ny': [], 'max_end_x': 0}
                            
                        # Sort and drop duplicates to allow for clean interpolation
                        p_traj_sorted = phase_df.dropna(subset=['normalized_distance', 'norm_x', 'norm_y']).sort_values(by='normalized_distance').drop_duplicates(subset=['normalized_distance'])
                        
                        if len(p_traj_sorted) > 1:
                            interp_nx = np.interp(common_norm_dist, p_traj_sorted['normalized_distance'], p_traj_sorted['norm_x'])
                            interp_ny = np.interp(common_norm_dist, p_traj_sorted['normalized_distance'], p_traj_sorted['norm_y'])
                            
                            mode_data[mode][phase][run_name]['nx'].append(interp_nx)
                            mode_data[mode][phase][run_name]['ny'].append(interp_ny)
                            
                            end_x = p_traj_sorted['norm_end_x'].iloc[0]
                            mode_data[mode][phase][run_name]['max_end_x'] = max(mode_data[mode][phase][run_name]['max_end_x'], end_x)
                            
            except Exception as e:
                print(f"Skipping {file} due to error: {e}")

    # Define explicit order and colors
    phase_config = {
        'careful': 'tab:green',
        'normal': 'tab:orange',
        'aggressive': 'tab:red'
    }

    # Generate the plots for each mode
    for mode, phases_dict in mode_data.items():
        ordered_phases = [p for p in phase_config.keys() if p in phases_dict]
        
        # Track limits specifically for THIS mode's plot
        fig_min_x, fig_max_x = 0.0, 0.0
        fig_min_y, fig_max_y = 0.0, 0.0
        
        # We want exactly 3 columns (Careful, Normal, Aggressive)
        fig, axes = plt.subplots(1, 3, figsize=(18, 5), sharex=True, sharey=True)
        if not isinstance(axes, np.ndarray):
            axes = [axes]
        
        for i in range(3):
            ax = axes[i]
            if i < len(ordered_phases):
                phase = ordered_phases[i]
                color = phase_config[phase]
                phase_data = phases_dict[phase]
                
                local_max_end_x = 0
                label_added = False
                
                # Plot the mean line for EACH USER
                for user_name, user_data in phase_data.items():
                    if user_data['nx'] and user_data['ny']:
                        # Calculate the mean trajectory across all runs for this specific user
                        user_mean_nx = np.mean(user_data['nx'], axis=0)
                        user_mean_ny = np.mean(user_data['ny'], axis=0)
                        
                        ax.plot(user_mean_nx, user_mean_ny, color=color, linewidth=2, alpha=0.7, 
                                label="User Mean" if not label_added else "")
                        label_added = True
                        
                        # Expand figure limits based on plotted lines
                        fig_min_x = min(fig_min_x, np.min(user_mean_nx))
                        fig_max_x = max(fig_max_x, np.max(user_mean_nx))
                        fig_min_y = min(fig_min_y, np.min(user_mean_ny))
                        fig_max_y = max(fig_max_y, np.max(user_mean_ny))
                        
                        local_max_end_x = max(local_max_end_x, user_data['max_end_x'])
                
                # Expand X bounds to cover reference line
                fig_max_x = max(fig_max_x, local_max_end_x)
                
                # Markers and Reference Line
                ax.plot([0, local_max_end_x], [0, 0], 'k--', alpha=0.8, label='Reference')
                ax.scatter(0, 0, c='green', marker='o', s=50, zorder=5, label='Start', alpha=0.7, linestyle=':')
                ax.scatter(local_max_end_x, 0, c='red', marker='X', s=50, zorder=5, label='End', alpha=0.7, linestyle=':')
                
                ax.set_title(f"Phase: {phase.replace('_', ' ').title()}")
                ax.set_xlabel("Normalized X")
                if i == 0:
                    ax.set_ylabel("Normalized Y")
                ax.grid(True)
                
                # Force Legend location to top right with an opaque background
                ax.legend(loc='upper right', fontsize=9, framealpha=0.95, edgecolor='gray')
            else:
                ax.set_visible(False)
        
        # Calculate padding and apply safely to this specific plot
        rng_x = fig_max_x - fig_min_x
        pad_x = rng_x * 0.05 if rng_x != 0 else 1.0
        axes[0].set_xlim(fig_min_x - pad_x, fig_max_x + pad_x)
        
        rng_y = fig_max_y - fig_min_y
        pad_y_bottom = rng_y * 0.05 if rng_y != 0 else 0.01
        
        # FIXED absolute padding added to the top to accommodate the legend safely
        fixed_top_padding = 0.035
        axes[0].set_ylim(fig_min_y - pad_y_bottom, fig_max_y + fixed_top_padding)
        
        # Ensure aspect ratio is equal to accurately reflect deviation magnitude
        for ax in axes:
            ax.set_aspect('equal', adjustable='box')
            
        plt.tight_layout()
        
        save_path = os.path.join(output_dir, f"directories_mean_trajectories_{mode}.pdf")
        plt.savefig(save_path, bbox_inches='tight')
        plt.close(fig)
        print(f"Successfully generated comparison plot: {save_path}")

if __name__ == "__main__":
    plot_user_mean_trajectories()