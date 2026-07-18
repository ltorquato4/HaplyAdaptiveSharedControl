import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 1. Math & Metric Calculations
# ==========================================

def calculate_metrics(df):
    """
    Calculates orthogonal positional error and normalized distance traveled.
    Assumes df has: cursor_x, cursor_y, start_x, start_y, end_x, end_y
    """
    # Vectors from start to end (the ideal path)
    line_vec_x = df['end_x'] - df['start_x']
    line_vec_y = df['end_y'] - df['start_y']
    line_len_sq = line_vec_x**2 + line_vec_y**2
    line_len = np.sqrt(line_len_sq)

    # Vectors from start to current cursor position
    point_vec_x = df['cursor_x'] - df['start_x']
    point_vec_y = df['cursor_y'] - df['start_y']

    # 1. Positional Error (Orthogonal distance to the ideal line)
    # Using cross product magnitude formula: |(P - A) x (B - A)| / |B - A|
    cross_prod = np.abs(point_vec_x * line_vec_y - point_vec_y * line_vec_x)
    # Avoid division by zero if start and end are the same
    df['orthogonal_error'] = np.where(line_len == 0, 0, cross_prod / line_len)

    # 2. Normalized Distance (Projection onto the ideal line)
    # Dot product: ((P - A) * (B - A)) / |B - A|^2
    dot_prod = (point_vec_x * line_vec_x) + (point_vec_y * line_vec_y)
    df['normalized_distance'] = np.where(line_len_sq == 0, 0, dot_prod / line_len_sq)

    return df

# ==========================================
# 2. Plotting Functions
# ==========================================

def generate_plots(df, controller, behavior, output_dir):
    """
    Generates the three required plots for a given subset of data.
    """
    # Create distinct output directory based on controller type
    save_dir = os.path.join(output_dir, controller)
    os.makedirs(save_dir, exist_ok=True)
    
    prefix = f"{controller}_{behavior}"
    
    # Identify unique trajectories to plot them as distinct lines
    trajectories = df['trajectory_id'].unique()

    # --- Plot 1: 2D Cursor Trajectory ---
    plt.figure(figsize=(8, 6))
    for traj in trajectories:
        traj_data = df[df['trajectory_id'] == traj]
        plt.plot(traj_data['cursor_x'], traj_data['cursor_y'], alpha=0.5)
    
    # Overlay Start and End Markers (assuming they are constant for the set)
    plt.scatter(df['start_x'].iloc[0], df['start_y'].iloc[0], c='green', marker='o', s=100, label='Start', zorder=5)
    plt.scatter(df['end_x'].iloc[0], df['end_y'].iloc[0], c='red', marker='X', s=100, label='End', zorder=5)
    
    plt.title(f"2D Cursor Trajectory\nController: {controller.title()} | Phase: {behavior.title()}")
    plt.xlabel("Cursor X")
    plt.ylabel("Cursor Y")
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(save_dir, f"{prefix}_2d_trajectory.png"))
    plt.close()

    # --- Plot 2: Positional Error over Normalized Distance ---
    plt.figure(figsize=(8, 6))
    for traj in trajectories:
        traj_data = df[df['trajectory_id'] == traj]
        # Sort by normalized distance to ensure clean line plotting
        traj_data = traj_data.sort_values(by='normalized_distance')
        plt.plot(traj_data['normalized_distance'], traj_data['orthogonal_error'], alpha=0.5)
    
    plt.title(f"Positional Error vs Normalized Distance\nController: {controller.title()} | Phase: {behavior.title()}")
    plt.xlabel("Normalized Distance (0 = Start, 1 = End)")
    plt.ylabel("Orthogonal Error")
    plt.grid(True)
    plt.savefig(os.path.join(save_dir, f"{prefix}_positional_error.png"))
    plt.close()

    # --- Plot 3: Velocity Profiles ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for traj in trajectories:
        traj_data = df[df['trajectory_id'] == traj]
        ax1.plot(traj_data['timestamp'], traj_data['haply_vel_x'], alpha=0.5)
        ax2.plot(traj_data['timestamp'], traj_data['haply_vel_y'], alpha=0.5)
    
    ax1.set_title(f"Velocity Profile (X-axis)\nController: {controller.title()} | Phase: {behavior.title()}")
    ax1.set_ylabel("Velocity X")
    ax1.grid(True)

    ax2.set_title("Velocity Profile (Y-axis)")
    ax2.set_xlabel("Timestamp")
    ax2.set_ylabel("Velocity Y")
    ax2.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f"{prefix}_velocity_profiles.png"))
    plt.close()

# ==========================================
# 3. Main Execution Workflow
# ==========================================

def main(data_directory="data", output_directory="analysis_plots"):
    # Find all CSV files in the directory
    csv_files = glob.glob(os.path.join(data_directory, "**", "*.csv"), recursive=True)
    
    all_data = []
    
    for i, file in enumerate(csv_files):
        df = pd.read_csv(file)
        df['trajectory_id'] = f"traj_{i}"
        
        # Calculate spatial metrics for this specific trajectory
        df = calculate_metrics(df)
        all_data.append(df)
        
    if not all_data:
        print("No CSV files found in the specified directory.")
        return

    # Combine all trajectories into one large DataFrame
    master_df = pd.concat(all_data, ignore_index=True)

    # Use the updated column names
    controllers = master_df['study_controller_mode'].dropna().unique()
    behaviors = master_df['study_phase'].dropna().unique()

    # Iterate through Fixed vs Adaptive
    for controller in controllers:
        controller_df = master_df[master_df['study_controller_mode'] == controller]
        
        # 1. Generate plots OVER ALL trajectories for this controller
        print(f"Generating aggregated plots for {controller} controller...")
        generate_plots(controller_df, controller, "all_phases", output_directory)
        
        # 2. Generate plots separated by the distinct behavioral modes
        for behavior in behaviors:
            behavior_df = controller_df[controller_df['study_phase'] == behavior]
            
            if not behavior_df.empty:
                print(f"Generating plots for {controller} controller - {behavior} phase...")
                generate_plots(behavior_df, controller, behavior, output_directory)

if __name__ == "__main__":
    main(data_directory="../processed_logs", output_directory="../plots/trajectory_plots")