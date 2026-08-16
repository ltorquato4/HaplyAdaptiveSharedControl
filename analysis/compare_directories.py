import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.ticker import MultipleLocator
import matplotlib.transforms as mtransforms

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
    """Plots the mean aligned trajectory of each user per controller mode and phase using discrete binning."""
    os.makedirs(output_dir, exist_ok=True)
    
    # Define 101 bin edges to create 100 discrete progress bins (0% to 100%)
    bin_edges = np.linspace(0.0, 1.0, 101)
    
    # Nested dict structure: {mode: {phase: {run_name: {'ndist': [], 'nx': [], 'ny': [], 'max_end_x': 0}}}}
    mode_data = {}
    
    subdirectories = [f.path for f in os.scandir(base_data_dir) if f.is_dir()]
    
    if not subdirectories:
        print(f"No subdirectories found in {base_data_dir}")
        return

    # Create a unique color for each participant (run_name)
    unique_users = sorted([os.path.basename(subdir) for subdir in subdirectories])
    
    # Choose a colormap that can handle the number of users (tab20 is good for up to 20, otherwise use a continuous one)
    cmap = cm.get_cmap('tab20') if len(unique_users) <= 20 else cm.get_cmap('nipy_spectral')
    user_colors = {user: cmap(i / len(unique_users)) for i, user in enumerate(unique_users)}

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
                            mode_data[mode][phase][run_name] = {'ndist': [], 'nx': [], 'ny': [], 'max_end_x': 0}
                            
                        # Keep raw data instead of interpolating
                        valid_data = phase_df.dropna(subset=['normalized_distance', 'norm_x', 'norm_y'])
                        
                        if not valid_data.empty:
                            mode_data[mode][phase][run_name]['ndist'].extend(valid_data['normalized_distance'].tolist())
                            mode_data[mode][phase][run_name]['nx'].extend(valid_data['norm_x'].tolist())
                            mode_data[mode][phase][run_name]['ny'].extend(valid_data['norm_y'].tolist())
                            
                            end_x = valid_data['norm_end_x'].iloc[0]
                            mode_data[mode][phase][run_name]['max_end_x'] = max(mode_data[mode][phase][run_name]['max_end_x'], end_x)
                            
            except Exception as e:
                print(f"Skipping {file} due to error: {e}")

    # Explicit order for the subplots
    ordered_phases_list = ['careful', 'normal', 'aggressive']

    # Generate the plots for each mode
    for mode, phases_dict in mode_data.items():
        ordered_phases = [p for p in ordered_phases_list if p in phases_dict]
        
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
                phase_data = phases_dict[phase]
                
                local_max_end_x = 0
                
                # Plot the mean line for EACH USER using discrete bins
                for user_name, user_data in phase_data.items():
                    if user_data['ndist']:
                        user_df = pd.DataFrame({
                            'ndist': user_data['ndist'], 
                            'nx': user_data['nx'], 
                            'ny': user_data['ny']
                        })
                        
                        # Filter to bounds and bin
                        user_df = user_df[(user_df['ndist'] >= 0.0) & (user_df['ndist'] <= 1.0)].copy()
                        user_df['bin'] = pd.cut(user_df['ndist'], bin_edges, labels=False, include_lowest=True)
                        
                        # Calculate mean within discrete bins
                        user_summary = user_df.groupby('bin')[['nx', 'ny']].mean().dropna()
                        
                        if not user_summary.empty:
                            user_mean_nx = user_summary['nx'].to_numpy()
                            user_mean_ny = user_summary['ny'].to_numpy()
                            
                            # Use the dynamically generated color for this specific user
                            user_color = user_colors[user_name]
                            
                            ax.plot(user_mean_nx, user_mean_ny, color=user_color, linewidth=2, alpha=0.7)
                            
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
                
                ax.set_title(f"{phase.replace('_', ' ').title()}")
                ax.set_xlabel("X [m]")
                if i == 0:
                    ax.set_ylabel("Y [m]")
                
                ax.yaxis.set_major_locator(MultipleLocator(0.01))
                ax.xaxis.set_major_locator(MultipleLocator(0.02))
                ax.grid(True)
            else:
                ax.set_visible(False)
        
        # Calculate padding and apply safely to this specific plot
        rng_x = fig_max_x - fig_min_x
        pad_x = rng_x * 0.05 if rng_x != 0 else 1.0
        axes[0].set_xlim(fig_min_x - pad_x, fig_max_x + pad_x)
        
        rng_y = fig_max_y - fig_min_y
        pad_y = rng_y * 0.05 if rng_y != 0 else 0.01
        axes[0].set_ylim(fig_min_y - pad_y, fig_max_y + pad_y)
        
        # Ensure aspect ratio is equal to accurately reflect deviation magnitude
        for ax in axes:
            ax.set_aspect('equal', adjustable='box')
            
        plt.tight_layout()
        
        handles, labels = axes[0].get_legend_handles_labels()
        if handles:
            # Create a transform that anchors to the bottom center (0.5, 0.0) but offsets by a fixed 30 points downwards
            offset = mtransforms.ScaledTranslation(0, -30/72., fig.dpi_scale_trans)
            fixed_offset_trans = axes[1].transAxes + offset
            
            # Attach the modified legend using the fixed physical transform
            axes[1].legend(handles, labels, loc='upper center', 
                           bbox_to_anchor=(0.5, 0.0), bbox_transform=fixed_offset_trans, 
                           ncol=3, fontsize=12, framealpha=0.95, edgecolor='gray')
        
        save_path = os.path.join(output_dir, f"directories_mean_trajectories_{mode}.pdf")
        
        # bbox_inches='tight' will crop away the large empty gaps caused by set_aspect('equal')
        plt.savefig(save_path, bbox_inches='tight')
        plt.close(fig)
        print(f"Successfully generated comparison plot: {save_path}")

if __name__ == "__main__":
    plot_user_mean_trajectories()