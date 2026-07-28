import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def calculate_metrics(df):
    """Calculates orthogonal error and normalized distance."""
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
    
    return df

def plot_directories_error_by_mode(base_data_dir="../processed_logs", output_dir="../plots/comparison_plots"):
    """Plots mean positional errors separated by mode and phase across all directories."""
    os.makedirs(output_dir, exist_ok=True)
    common_norm_dist = np.linspace(0, 1, 500)
    
    # Nested dict structure: {mode: {phase: {run_name: [interpolated_errors]}}}
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
                
                if 'study_controller_mode' not in df.columns or 'study_phase' not in df.columns:
                    continue
                
                # Standardize strings
                df['study_controller_mode'] = df['study_controller_mode'].astype(str).str.strip().str.lower()
                df['study_phase'] = df['study_phase'].astype(str).str.strip().str.lower()
                df = calculate_metrics(df)
                
                # Group by mode
                for mode, mode_df in df.groupby('study_controller_mode'):
                    if mode not in mode_data:
                        mode_data[mode] = {'all_phases': {}}
                        
                    if run_name not in mode_data[mode]['all_phases']:
                        mode_data[mode]['all_phases'][run_name] = []
                        
                    # 1. Store ALL PHASES data for this file
                    traj_sorted = mode_df.sort_values(by='normalized_distance')
                    if len(traj_sorted) > 1:
                        interp = np.interp(
                            common_norm_dist, 
                            traj_sorted['normalized_distance'], 
                            traj_sorted['orthogonal_error']
                        )
                        mode_data[mode]['all_phases'][run_name].append(interp)
                    
                    # 2. Store INDIVIDUAL PHASE data for this file
                    for phase, phase_df in mode_df.groupby('study_phase'):
                        if phase not in mode_data[mode]:
                            mode_data[mode][phase] = {}
                        if run_name not in mode_data[mode][phase]:
                            mode_data[mode][phase][run_name] = []
                            
                        p_traj_sorted = phase_df.sort_values(by='normalized_distance')
                        if len(p_traj_sorted) > 1:
                            interp = np.interp(
                                common_norm_dist, 
                                p_traj_sorted['normalized_distance'], 
                                p_traj_sorted['orthogonal_error']
                            )
                            mode_data[mode][phase][run_name].append(interp)
                            
            except Exception as e:
                print(f"Skipping {file} due to error: {e}")

    # Generate the plots
    for mode, phases_dict in mode_data.items():
        # Get individual phases and append 'all_phases' at the bottom
        individual_phases = sorted([p for p in phases_dict.keys() if p != 'all_phases'])
        plot_phases = individual_phases + ['all_phases']
        
        # Create stacked subplots
        fig, axes = plt.subplots(len(plot_phases), 1, figsize=(10, 2.5 * len(plot_phases)), sharex=True, sharey=True)
        
        if len(plot_phases) == 1:
            axes = [axes]
            
        fig.suptitle(f"Mean Positional Error Comparison (No Variance)\nController Mode: {mode.title()}", fontsize=14)
        
        # Populate each subplot
        for i, phase in enumerate(plot_phases):
            ax = axes[i]
            ax.plot([0, 1], [0, 0], 'k--', alpha=0.5, linewidth=1.5, zorder=1)
            ax.grid(True)
            ax.set_ylabel("Error")
            
            for run_name, errors in phases_dict[phase].items():
                if errors:
                    # Calculate mean line only, skipping variance fills
                    mean_err = np.mean(errors, axis=0)
                    ax.plot(common_norm_dist, mean_err, linewidth=2)  # Label removed here
            
            # Label the subplot with the phase name inline
            phase_label = "All Phases" if phase == 'all_phases' else phase.replace('_', ' ').title()
            ax.text(0.02, 0.85, phase_label, transform=ax.transAxes, fontsize=11, fontweight='bold', va='top')
            
            # The legend block has been removed completely
                
        # Global limits and formatting
        axes[-1].set_xlabel("Position along Reference Trajectory (Normalized)")
        axes[0].set_xlim(0, 1)
        
        plt.tight_layout()
        fig.subplots_adjust(top=0.92)
        
        save_path = os.path.join(output_dir, f"directories_error_comparison_{mode}.pdf")
        plt.savefig(save_path, bbox_inches='tight')
        plt.close(fig)
        print(f"Successfully generated comparison plot: {save_path}")

if __name__ == "__main__":
    plot_directories_error_by_mode()