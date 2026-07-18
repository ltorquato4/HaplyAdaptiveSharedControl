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

    cross_prod = np.abs(point_vec_x * line_vec_y - point_vec_y * line_vec_x)
    df['orthogonal_error'] = np.where(line_len == 0, 0, cross_prod / line_len)

    dot_prod = (point_vec_x * line_vec_x) + (point_vec_y * line_vec_y)
    df['normalized_distance'] = np.where(line_len_sq == 0, 0, dot_prod / line_len_sq)

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

def generate_plots(df, controller, behavior, output_dir, limits, aggregate_only=False):
    save_dir = os.path.join(output_dir, controller, behavior)
    os.makedirs(save_dir, exist_ok=True)
    
    trajectories = df['file_stem'].unique()

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
            plt.title(f"2D Cursor Trajectory\nController: {controller.title()} | Phase: {behavior.title()} | Run: {traj}")
            plt.xlabel("Cursor X")
            plt.ylabel("Cursor Y")
            plt.xlim(limits['x_2d'])
            plt.ylim(limits['y_2d'])
            plt.legend()
            plt.grid(True)
            plt.savefig(os.path.join(save_dir, f"{prefix}_2d_trajectory.pdf"))
            plt.close()

            # 2. Positional Error
            plt.figure(figsize=(8, 6))
            traj_data_sorted = traj_data.sort_values(by='normalized_distance')
            plt.plot(traj_data_sorted['normalized_distance'], traj_data_sorted['orthogonal_error'])
            plt.title(f"Positional Error vs Normalized Distance\nController: {controller.title()} | Phase: {behavior.title()} | Run: {traj}")
            plt.xlabel("Normalized Distance (0 = Start, 1 = End)")
            plt.ylabel("Orthogonal Error")
            plt.xlim(0, 1)
            plt.ylim(limits['error'])
            plt.grid(True)
            plt.savefig(os.path.join(save_dir, f"{prefix}_positional_error.pdf"))
            plt.close()

            # 3. Velocity Profiles
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
            ax1.plot(traj_data['timestamp'], traj_data['haply_vel_x'])
            ax2.plot(traj_data['timestamp'], traj_data['haply_vel_y'])
            ax1.set_title(f"Velocity Profile\nController: {controller.title()} | Phase: {behavior.title()} | Run: {traj}")
            ax1.set_ylabel("Velocity X")
            ax1.set_xlim(limits['time'])
            ax1.set_ylim(limits['vel_x'])
            ax1.grid(True)
            ax2.set_xlabel("Timestamp")
            ax2.set_ylabel("Velocity Y")
            ax2.set_ylim(limits['vel_y'])
            ax2.grid(True)
            plt.tight_layout()
            plt.savefig(os.path.join(save_dir, f"{prefix}_velocity_profiles.pdf"))
            plt.close()

    # ----------------------------------------
    # AGGREGATED PLOTS
    # ----------------------------------------
    prefix_all = f"{controller}_{behavior}_all"
    title_phase = behavior.replace('_', ' ').title()
    
    # 1. 2D Cursor Trajectory (all)
    plt.figure(figsize=(8, 6))
    for idx, traj in enumerate(trajectories):
        traj_data = df[df['file_stem'] == traj]
        plt.plot(traj_data['cursor_x'], traj_data['cursor_y'])
        
        # Only add the label for the legend on the very first loop iteration
        l_start = 'Start' if idx == 0 else ""
        l_end = 'End' if idx == 0 else ""
        
        # Plot start and end points for EVERY trajectory
        plt.scatter(traj_data['start_x'].iloc[0], traj_data['start_y'].iloc[0], c='green', marker='o', s=100, label=l_start, zorder=5, alpha=0.7)
        plt.scatter(traj_data['end_x'].iloc[0], traj_data['end_y'].iloc[0], c='red', marker='X', s=100, label=l_end, zorder=5, alpha=0.7)
        
    plt.title(f"2D Cursor Trajectory\nController: {controller.title()} | {title_phase}")
    plt.xlabel("Cursor X")
    plt.ylabel("Cursor Y")
    plt.xlim(limits['x_2d'])
    plt.ylim(limits['y_2d'])
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(save_dir, f"{prefix_all}_2d_trajectory.pdf"))
    plt.close()

    # 2. Positional Error (all)
    plt.figure(figsize=(8, 6))
    
    # Create a uniform grid from 0 to 1 to align all trajectories for averaging
    common_norm_dist = np.linspace(0, 1, 500)
    interpolated_errors = []

    for traj in trajectories:
        traj_data = df[df['file_stem'] == traj]
        traj_data_sorted = traj_data.sort_values(by='normalized_distance')
        
        # Plot the individual trajectory
        plt.plot(traj_data_sorted['normalized_distance'], traj_data_sorted['orthogonal_error'], alpha=0.6)
        
        # Interpolate the trajectory's error onto the common grid
        if len(traj_data_sorted) > 1:
            interp_error = np.interp(
                common_norm_dist, 
                traj_data_sorted['normalized_distance'], 
                traj_data_sorted['orthogonal_error']
            )
            interpolated_errors.append(interp_error)

    # Calculate and plot the mean trajectory
    if interpolated_errors:
        mean_error = np.mean(interpolated_errors, axis=0)
        plt.plot(common_norm_dist, mean_error, color='black', linewidth=3, linestyle='--', label='Mean Error', zorder=10)
        plt.legend()

    plt.title(f"Positional Error vs Normalized Distance\nController: {controller.title()} | {title_phase}")
    plt.xlabel("Normalized Distance (0 = Start, 1 = End)")
    plt.ylabel("Orthogonal Error")
    plt.xlim(0, 1)
    plt.ylim(limits['error'])
    plt.grid(True)
    plt.savefig(os.path.join(save_dir, f"{prefix_all}_positional_error.pdf"))
    plt.close()

    # 3. Velocity Profiles (all)
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
    ax2.set_xlabel("Timestamp")
    ax2.set_ylabel("Velocity Y")
    ax2.set_ylim(limits['vel_y'])
    ax2.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f"{prefix_all}_velocity_profiles.pdf"))
    plt.close()

# ==========================================
# 3. Main Execution Workflow
# ==========================================

def main(data_directory="data", output_directory="analysis_plots"):
    csv_files = glob.glob(os.path.join(data_directory, "**", "*.csv"), recursive=True)
    all_data = []
    
    for file in csv_files:
        df = pd.read_csv(file)
        
        if 'study_controller_mode' in df.columns: df['study_controller_mode'] = df['study_controller_mode'].astype(str).str.strip().str.lower()
        if 'study_phase' in df.columns: df['study_phase'] = df['study_phase'].astype(str).str.strip().str.lower()
            
        df['file_stem'] = Path(file).stem
        df = calculate_metrics(df)
        all_data.append(df)
        
    if not all_data:
        print("No CSV files found in the specified directory.")
        return

    master_df = pd.concat(all_data, ignore_index=True)
    
    limits = {
        'x_2d': get_padded_limits([master_df['cursor_x'], master_df['start_x'], master_df['end_x']]),
        'y_2d': get_padded_limits([master_df['cursor_y'], master_df['start_y'], master_df['end_y']]),
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

if __name__ == "__main__":
    main(data_directory="../processed_logs", output_directory="../plots/trajectory_plots")