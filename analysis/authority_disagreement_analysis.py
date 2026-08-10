import os
import glob
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# ==========================================
# 1. Parsing and Mathematical Logic
# ==========================================
def parse_and_calculate_inputs(df):
    kh_x1, kh_x2, kh_y1, kh_y2 = [], [], [], []
    
    for json_str in df.get('K_h', []):
        if pd.isna(json_str):
            kh_x1.append(np.nan); kh_x2.append(np.nan)
            kh_y1.append(np.nan); kh_y2.append(np.nan)
            continue
        try:
            data = json.loads(json_str)
            if isinstance(data, list) and len(data) >= 8 and not isinstance(data[0], list):
                kh_x1.append(data[0])  # k_x1
                kh_x2.append(data[1])  # k_x2
                kh_y1.append(data[6])  # k_y1
                kh_y2.append(data[7])  # k_y2
            else:
                kh_x1.append(np.nan); kh_x2.append(np.nan)
                kh_y1.append(np.nan); kh_y2.append(np.nan)
        except (json.JSONDecodeError, TypeError, IndexError):
            kh_x1.append(np.nan); kh_x2.append(np.nan)
            kh_y1.append(np.nan); kh_y2.append(np.nan)

    df['Kh_x1'] = kh_x1
    df['Kh_x2'] = kh_x2
    df['Kh_y1'] = kh_y1
    df['Kh_y2'] = kh_y2

    def parse_input_array(val):
        if pd.isna(val): return np.nan
        try:
            arr = json.loads(val) if isinstance(val, str) else val
            if isinstance(arr, list) and len(arr) >= 2: return np.sqrt(arr[0]**2 + arr[1]**2)
            elif isinstance(arr, (int, float)): return abs(arr)
            return np.nan
        except: return np.nan
        
    df['u_h_mag'] = df['u_h'].apply(parse_input_array) if 'u_h' in df.columns else np.nan
    df['u_a_mag'] = df['u_a'].apply(parse_input_array) if 'u_a' in df.columns else np.nan

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
def add_global_phase_labels(ax, df):
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
            # Increased gap to 60.0s to allow separate file intervals to comfortably span into aggregated visual blocks
            if row['phase'] == last['phase'] and row['min'] <= last['max'] + 60.0: 
                last['max'] = max(last['max'], row['max'])
            else:
                merged_blocks.append(row.to_dict())
                
    trans = ax.get_xaxis_transform()
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
        
        y_arrow = -0.06 - (level * 0.08)
        y_text = -0.09 - (level * 0.08)
        
        if p_start > df['timestamp'].min():
            ax.axvline(x=p_start, color='black', linestyle='--', linewidth=1.2, alpha=0.6)
        if p_end < df['timestamp'].max():
            ax.axvline(x=p_end, color='black', linestyle='--', linewidth=1.2, alpha=0.6)
            
        ax.annotate('', xy=(p_start, y_arrow), xytext=(p_end, y_arrow),
                    xycoords=trans, textcoords=trans,
                    arrowprops=dict(arrowstyle='<|-|>', color='black', shrinkA=0, shrinkB=0),
                    annotation_clip=False)
                    
        ax.text(p_mid, y_text, phase_name, transform=trans,
                ha='center', va='top', fontsize=11, color='black', clip_on=False)
                
    return len(levels)

def generate_aggregated_plots(df, controller, output_dir, limits):
    save_dir = os.path.join(output_dir, controller)
    os.makedirs(save_dir, exist_ok=True)
    
    colors = {'x1': 'tab:blue', 'x2': 'tab:orange', 'y1': 'tab:green', 'y2': 'tab:red'}

    # ----------------------------------------
    # Plot 1: Kh Evolution (Aggregated)
    # ----------------------------------------
    fig, ax = plt.subplots(figsize=(12, 7))
    
    # Sort the ENTIRE dataframe for this controller chronologically
    # This prevents Matplotlib from treating separate files as separated segments.
    traj_data = df.sort_values('timestamp')
    
    if not traj_data[['Kh_x1', 'Kh_x2', 'Kh_y1', 'Kh_y2']].isna().all().all():
        # Drop NaNs to connect lines directly across any empty rows
        valid_x1 = traj_data.dropna(subset=['Kh_x1'])
        ax.plot(valid_x1['timestamp'], valid_x1['Kh_x1'], color=colors['x1'], label=r'$k_{x_1}$')
        
        valid_x2 = traj_data.dropna(subset=['Kh_x2'])
        ax.plot(valid_x2['timestamp'], valid_x2['Kh_x2'], color=colors['x2'], label=r'$k_{x_2}$')
        
        valid_y1 = traj_data.dropna(subset=['Kh_y1'])
        ax.plot(valid_y1['timestamp'], valid_y1['Kh_y1'], color=colors['y1'], label=r'$k_{y_1}$')
        
        valid_y2 = traj_data.dropna(subset=['Kh_y2'])
        ax.plot(valid_y2['timestamp'], valid_y2['Kh_y2'], color=colors['y2'], label=r'$k_{y_2}$')
            
    ax.set_ylabel("Estimated $K_h$ Components")
    ax.set_xlim(limits['time'])
    ax.set_ylim(limits['kh'])
    ax.grid(True)
    
    num_levels = add_global_phase_labels(ax, df)
    ax.set_xlabel("Time", labelpad=35 + (num_levels * 18))
    
    ax.legend(loc='upper right')
    plt.savefig(os.path.join(save_dir, f"{controller}_all_Kh.pdf"), bbox_inches='tight')
    plt.close()

# ==========================================
# 3. Main Execution Workflow
# ==========================================
def process_dataframe(df, file_stem):
    if 'study_controller_mode' in df.columns: 
        df['study_controller_mode'] = df['study_controller_mode'].astype(str).str.strip().str.lower()
    if 'study_phase' in df.columns: 
        df['study_phase'] = df['study_phase'].astype(str).str.strip().str.lower()
        
    df = parse_and_calculate_inputs(df)
    df['file_stem'] = file_stem
    return df

def main(data_directory="data", base_output_dir="authority_plots"):
    csv_files = glob.glob(os.path.join(data_directory, "**", "*.csv"), recursive=True)
    
    if not csv_files:
        print("No CSV files found in the specified directory.")
        return

    all_data = []
    for file in csv_files:
        try:
            df = pd.read_csv(file)
            required_cols = ['cursor_x', 'cursor_y', 'end_x', 'end_y']
            if all(col in df.columns for col in required_cols):
                df = df.dropna(subset=required_cols)
                
            if not df.empty:
                all_data.append(process_dataframe(df, Path(file).stem))
        except pd.errors.EmptyDataError:
            continue
        except Exception as e:
            print(f"Skipping {file} due to error: {e}")

    if not all_data:
        print("No valid data could be processed.")
        return
        
    master_df = pd.concat(all_data, ignore_index=True)
    
    global_limits = {
        'kh': get_padded_limits([master_df['Kh_x1'], master_df['Kh_x2'], master_df['Kh_y1'], master_df['Kh_y2']]),
        'u_mag': get_padded_limits([master_df['u_h_mag'], master_df['u_a_mag']])
    }

    controllers = master_df['study_controller_mode'].dropna().unique()

    for controller in controllers:
        controller_df = master_df[master_df['study_controller_mode'] == controller]
        
        controller_limits = global_limits.copy()
        controller_limits['time'] = get_padded_limits([controller_df['timestamp']], pad=0)
        
        print(f"Generating aggregated plot for {controller.upper()} Controller...")
        generate_aggregated_plots(controller_df, controller, base_output_dir, controller_limits)

if __name__ == "__main__":
    main(data_directory="../processed_logs", base_output_dir="../plots/authority_plots")