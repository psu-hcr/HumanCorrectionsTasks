import os
import pandas as pd

def normalize_time(df):
    max_seq = df['field.poseStamped.header.seq'].max()
    df['normalized_time'] = df['field.poseStamped.header.seq'] / max_seq
    return df

def process_text_file(file_path):
    # Read the text file into a DataFrame
    df = pd.read_csv(file_path)

    # Normalize time based on sequence number
    df = normalize_time(df)

    return df

def calculate_average_positions(data_frames):
    # Create an empty DataFrame to store the averaged positions
    avg_df = pd.DataFrame(columns=['normalized_time', 'avg_position_x', 'avg_position_y', 'avg_position_z'])

    # Get the unique normalized time points from all DataFrames
    unique_times = sorted(set(time for df in data_frames.values() for time in df['normalized_time']))

    # Calculate the average pose positions for each normalized time point
    for time in unique_times:
        relevant_dfs = [df[df['normalized_time'] == time] for df in data_frames.values() if time in df['normalized_time'].values]

        avg_position_x = sum(df['field.poseStamped.pose.position.x'].iloc[0] for df in relevant_dfs) / len(relevant_dfs)
        avg_position_y = sum(df['field.poseStamped.pose.position.y'].iloc[0] for df in relevant_dfs) / len(relevant_dfs)
        avg_position_z = sum(df['field.poseStamped.pose.position.z'].iloc[0] for df in relevant_dfs) / len(relevant_dfs)

        avg_df = avg_df.append({
            'normalized_time': time,
            'avg_position_x': avg_position_x,
            'avg_position_y': avg_position_y,
            'avg_position_z': avg_position_z
        }, ignore_index=True)

    return avg_df

def save_to_csv(df, directory_path):
    csv_path = os.path.join(directory_path, 'averaged_positions.csv')
    df.to_csv(csv_path, index=False)
    print(f"Averaged positions saved to: {csv_path}")

def main(directory_path):
    # Get a list of all text files in the specified directory
    text_files = [f for f in os.listdir(directory_path) if f.endswith('.txt')]

    # Process each text file and store the DataFrames in a dictionary
    data_frames = {}
    for file in text_files:
        file_path = os.path.join(directory_path, file)
        df = process_text_file(file_path)
        data_frames[file] = df

        # Print the normalized DataFrame
        #print(f"Normalized DataFrame for {file}:\n{df[['field.poseStamped.header.seq', 'normalized_time']]}")
        print("done")
        #print("\n" + "="*50 + "\n")

    # Calculate average pose positions for each normalized time point
    avg_positions_df = calculate_average_positions(data_frames)

    # Save the averaged positions to a new CSV file
    save_to_csv(avg_positions_df, directory_path)

if __name__ == "__main__":
    directory_path = "/your_path_of_folder"  # This folder contains non-interaction txt files 
    main(directory_path)
