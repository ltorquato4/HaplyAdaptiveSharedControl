import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def main(data_directory="../logs", output_directory="../plots/questionnaire_plots"):
    """
    Extracts subjective questionnaire data from all participant logs 
    and generates individual side-by-side boxplots matching the paper/LaTeX layout style.
    """
    os.makedirs(output_directory, exist_ok=True)
    
    # 1. Gather all questionnaire data from the raw logs directory
    search_pattern = os.path.join(data_directory, "**", "questionnaire", "questionnaire.csv")
    questionnaire_files = glob.glob(search_pattern, recursive=True)
    
    if not questionnaire_files:
        print(f"No questionnaire.csv files found in {data_directory}")
        return

    extracted_data = []

    # 2. Extract and map the subjective ratings
    for file_path in questionnaire_files:
        try:
            df = pd.read_csv(file_path)
            if df.empty:
                continue
            
            row = df.iloc[0] # One row per participant questionnaire
            
            # Identify the mapping of Controller A and Controller B
            mode_a = str(row.get("controller_a_mode")).strip().lower()
            mode_b = str(row.get("controller_b_mode")).strip().lower()
            
            # Categories and their concise display titles
            rating_categories = [
                ("sense_of_agency_rating", "Sense of Agency"),
                ("assistive_interaction_rating", "Assistive Interaction"),
                ("user_experience_rating", "User Experience")
            ]
            
            # Map the ratings from A/B to their actual assigned modes
            for category_key, category_title in rating_categories:
                
                # Extract Controller A's rating
                rating_a = row.get(f"controller_a_{category_key}")
                if pd.notna(rating_a) and str(rating_a).strip() != "":
                    extracted_data.append({
                        "Participant": row.get("participant_id"),
                        "Mode": mode_a.title(),
                        "Category": category_title,
                        "Rating": float(rating_a)
                    })
                
                # Extract Controller B's rating
                rating_b = row.get(f"controller_b_{category_key}")
                if pd.notna(rating_b) and str(rating_b).strip() != "":
                    extracted_data.append({
                        "Participant": row.get("participant_id"),
                        "Mode": mode_b.title(),
                        "Category": category_title,
                        "Rating": float(rating_b)
                    })

        except Exception as e:
            print(f"Error processing {file_path}: {e}")

    if not extracted_data:
        print("No valid subjective ratings could be extracted.")
        return

    # Convert to DataFrame
    plot_df = pd.DataFrame(extracted_data)
    categories = plot_df["Category"].unique()
    modes = ["Fixed", "Adaptive"]  # Explicit order

    # 3. Create Subplot Row (1 x N Grid)
    fig, axes = plt.subplots(1, len(categories), figsize=(4 * len(categories), 4.5), sharey=True)
    if not isinstance(axes, np.ndarray):
        axes = [axes]

    # Customize Boxplot Aesthetics to match reference image (Blue boxes, red median, red/black outliers)
    boxprops = dict(linestyle='-', linewidth=1.2, color='blue', facecolor='none')
    medianprops = dict(linestyle='-', linewidth=1.5, color='red')
    whiskerprops = dict(linestyle='--', linewidth=1.0, color='black')
    capprops = dict(linestyle='-', linewidth=1.0, color='black')
    flierprops = dict(marker='+', markerfacecolor='red', markeredgecolor='red', markersize=8)

    for i, cat in enumerate(categories):
        ax = axes[i]
        cat_df = plot_df[plot_df["Category"] == cat]
        
        # Prepare data list per mode for matplotlib boxplot
        data_to_plot = [cat_df[cat_df["Mode"] == mode]["Rating"].values for mode in modes]
        
        # Draw boxplots
        bplot = ax.boxplot(
            data_to_plot,
            patch_artist=True,  # allows color styling
            labels=modes,
            boxprops=boxprops,
            medianprops=medianprops,
            whiskerprops=whiskerprops,
            capprops=capprops,
            flierprops=flierprops,
            widths=0.4
        )
        
        # Formatting individual subplot
        ax.set_title(cat, fontweight='bold', fontsize=15, pad=10)
        ax.set_ylim(0.5, 5.5)
        ax.set_yticks([1, 2, 3, 4, 5])
        ax.grid(True, axis='y', linestyle=':', alpha=0.6)
        
        # Rotate X-tick labels like reference image
        ax.set_xticklabels(modes, rotation=45, ha='right', fontsize=14)
        
        if i == 0:
            ax.set_ylabel("Rating Scale", fontsize=14)

    plt.tight_layout()

    # Save to PDF matching your existing analysis pipeline
    output_filename = os.path.join(output_directory, "subjective_ratings_subplots.pdf")
    plt.savefig(output_filename, bbox_inches='tight')
    plt.close(fig)
    
    print(f"Successfully evaluated subjective ratings across {len(plot_df)} responses.")
    print(f" -> Saved side-by-side subplots to: {output_filename}")

if __name__ == "__main__":
    main()