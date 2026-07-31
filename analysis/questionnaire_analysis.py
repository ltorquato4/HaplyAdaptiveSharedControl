import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

def main(data_directory="../logs", output_directory="../plots/questionnaire_plots"):
    """
    Extracts subjective questionnaire data from all participant logs 
    and generates a grouped boxplot comparing controller modes.
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
            
            # The specific rating fields to extract based on run_experiment.py
            rating_categories = [
                "sense_of_agency_rating",
                "assistive_interaction_rating",
                "user_experience_rating"
            ]
            
            # Map the ratings from A/B to their actual assigned modes
            for category in rating_categories:
                
                # Extract Controller A's rating
                rating_a = row.get(f"controller_a_{category}")
                if pd.notna(rating_a) and str(rating_a).strip() != "":
                    extracted_data.append({
                        "Participant": row.get("participant_id"),
                        "Mode": mode_a.title(),
                        "Category": category.replace("_rating", "").replace("_", " ").title(),
                        "Rating": float(rating_a)
                    })
                
                # Extract Controller B's rating
                rating_b = row.get(f"controller_b_{category}")
                if pd.notna(rating_b) and str(rating_b).strip() != "":
                    extracted_data.append({
                        "Participant": row.get("participant_id"),
                        "Mode": mode_b.title(),
                        "Category": category.replace("_rating", "").replace("_", " ").title(),
                        "Rating": float(rating_b)
                    })

        except Exception as e:
            print(f"Error processing {file_path}: {e}")

    if not extracted_data:
        print("No valid subjective ratings could be extracted.")
        return

    # Convert to DataFrame for plotting
    plot_df = pd.DataFrame(extracted_data)

    # 3. Generate the Boxplot
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Create a grouped boxplot comparing controller modes for each category
    sns.boxplot(
        data=plot_df, 
        x="Category", 
        y="Rating", 
        hue="Mode", 
        palette="Set2",
        width=0.6,
        showmeans=True,
        meanprops={"marker":"o", "markerfacecolor":"white", "markeredgecolor":"black"},
        ax=ax
    )

    # Formatting the plot
    ax.set_ylabel("Rating (1 = Strongly Disagree, 5 = Strongly Agree)")
    ax.set_yticks([1, 2, 3, 4, 5])
    
    # Force Legend location outside the plot to avoid overlapping data
    ax.legend(title="Controller Mode", loc='upper left', bbox_to_anchor=(1, 1))
    ax.grid(axis='y', linestyle='--', alpha=0.7)
    
    plt.tight_layout()

    # Save to PDF matching your existing pipeline output format
    output_filename = os.path.join(output_directory, "global_subjective_ratings_boxplot.pdf")
    plt.savefig(output_filename, bbox_inches='tight')
    plt.close(fig)
    
    print(f"Successfully evaluated {len(plot_df)} total ratings.")
    print(f" -> Saved subjective boxplot to: {output_filename}")

if __name__ == "__main__":
    main()