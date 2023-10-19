import pandas as pd
import os
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import MinMaxScaler
import numpy as np

# Define a function to extract data from a text file and normalize the 'field.poseStamped.header.seq' column
def extract_and_normalize_data(file_path):
    if not os.path.isfile(file_path):
        print(f"File not found: {file_path}")
        return None

    # Read the text file into a DataFrame
    df = pd.read_csv(file_path, sep=',')

    # Normalize the 'field.poseStamped.header.seq' column
    scaler = MinMaxScaler()
    df['normalized_seq'] = scaler.fit_transform(df[['field.poseStamped.header.seq']])

    return df




def find_interaction_points(data, standard_data, threshold1=0.01, threshold2=0.005):
    interaction_start_points = []
    interaction_stop_points = []

    start_found = False
    for i in range(len(data)):
        if i >= len(standard_data):
            break  # Ensure we don't go out of bounds in the standard data

        diff_x = abs(data['field.poseStamped.pose.position.x'].iloc[i] - standard_data['field.poseStamped.pose.position.x'].iloc[i])
        diff_y = abs(data['field.poseStamped.pose.position.y'].iloc[i] - standard_data['field.poseStamped.pose.position.y'].iloc[i])
        diff_z = abs(data['field.poseStamped.pose.position.z'].iloc[i] - standard_data['field.poseStamped.pose.position.z'].iloc[i])

        if not start_found:
            if diff_x > threshold2 or diff_y > threshold2 or diff_z > threshold2:
                interaction_start_points.append(i)
                start_found = True
        else:
            if diff_x < threshold1 and diff_y < threshold1 and diff_z < threshold1:
                interaction_stop_points.append(i)
                start_found = False

    # If the number of start points is greater than the number of stop points, add the last data point as a stop point
    if len(interaction_start_points) > len(interaction_stop_points):
        interaction_stop_points.append(len(data) - 1)

    return interaction_start_points, interaction_stop_points

def process_text_file(file_path, standard_file_path, sampling_rate=1/500):
    if not os.path.isfile(file_path):
        print(f"File not found: {file_path}")
        return None

    # Read the text file into a DataFrame
    df = pd.read_csv(file_path, sep=',')

    # Read the standard data (assuming the standard data is in a file named "standard_CP.txt" in the same directory)
    #standard_file_path = os.path.join(os.path.dirname(file_path), "CP_pour_correct.txt")
    standard_df = pd.read_csv(standard_file_path, sep=',')

    # Ensure that the data frames have the same number of rows
    max_length = min(len(df), len(standard_df))
    df = df.iloc[:max_length]
    standard_df = standard_df.iloc[:max_length]

    # Find interaction points
    interaction_start_points, interaction_stop_points = find_interaction_points(df, standard_df)

    # Calculate total interaction time
    total_interaction_time = 0
    for start, stop in zip(interaction_start_points, interaction_stop_points):
        # Check for out-of-bounds indices
        if start < len(df) and stop < len(df):
            time_period = (stop - start) * sampling_rate
            total_interaction_time += time_period

    return df, interaction_start_points, interaction_stop_points, total_interaction_time
"""
# Directory containing your text files
directory_path_correct = '/home/jvp6149/Desktop/bagfiles_Pang/organized Tiral 9/peg_correct'

# List of file names you want to process, including "standard_CP.txt" and others
file_names_correct = ["CP_peg_correct.txt", "CP_peg_1_correct.txt", "CP_peg_2_correct.txt", "CP_peg_4.txt", "CP_peg_6.txt", "CP_peg_14.txt", "CP_peg_16.txt", "CP_peg_26.txt", "CP_peg_37.txt"]

for file_name in file_names_correct:
    file_path = os.path.join(directory_path_correct, file_name)
    data, interaction_start_points, interaction_stop_points, total_interaction_time = process_text_file(file_path)

    # Normalize the 'field.poseStamped.header.seq' column using Min-Max scaling
    scaler = MinMaxScaler()
    data['normalized_seq'] = scaler.fit_transform(data[['field.poseStamped.header.seq']])

    plt.figure(figsize=(10, 6))

    # Plot the relationship between 'field.poseStamped.pose.position.x' and 'normalized_seq' for each file
    plt.plot(data['normalized_seq'], data['field.poseStamped.pose.position.x'], label='Position X')
    plt.scatter(data['normalized_seq'].iloc[interaction_start_points], data['field.poseStamped.pose.position.x'].iloc[interaction_start_points], c='g', marker='o', label='Interaction Start')
    plt.scatter(data['normalized_seq'].iloc[interaction_stop_points], data['field.poseStamped.pose.position.x'].iloc[interaction_stop_points], c='r', marker='x', label='Interaction Stop')

    plt.xlabel('Normalized Sequence (0-1)')
    plt.ylabel('field.poseStamped.pose.position.x')
    plt.legend()
    plt.title(f'Relationship between Position X and Normalized Sequence - {file_name}')
    plt.show()

    print(f"File: {file_name}")
    print(f"Interaction Start Points: {interaction_start_points}")
    print(f"Interaction Stop Points: {interaction_stop_points}")
    print(f"Total Interaction Time: {total_interaction_time} seconds")
    print()

"""    
def calculate_average_interaction_time(directory_path):
    folder_names = os.listdir(directory_path)
    average_interaction_times = []
    std_deviation_interaction_times = []

    for folder_name in folder_names:
        folder_path = os.path.join(directory_path, folder_name)
        file_names = os.listdir(folder_path)

        interaction_times = []  # Store interaction times for each file in the folder

        for file_name in file_names:
            file_path = os.path.join(folder_path, file_name)
            standard_file_path = os.path.join(os.path.dirname(file_path), "CP_pour_correct.txt")
            data, interaction_start_points, interaction_stop_points, total_interaction_time = process_text_file(file_path, standard_file_path)
            interaction_times.append(total_interaction_time)

        # Calculate the average interaction time for this folder (excluding the standard file)
        interaction_times_without_standard = interaction_times[1:]  # Exclude the first (standard) file
        average_time = sum(interaction_times_without_standard) / len(interaction_times_without_standard)
        
        # Calculate the standard deviation of interaction times
        std_deviation_time = 0.2 * np.std(interaction_times_without_standard)

        average_interaction_times.append(average_time)
        std_deviation_interaction_times.append(std_deviation_time)

    return average_interaction_times, std_deviation_interaction_times, folder_names

def plot_average_interaction_times(directory_path):
    average_interaction_times, std_deviation_interaction_times, folder_names = calculate_average_interaction_time(directory_path)

    # Create a bar plot with error bars for average interaction times of the 5 folders
    plt.figure(figsize=(10, 6))
    plt.bar(folder_names, average_interaction_times, yerr=std_deviation_interaction_times, capsize=5)
    plt.xlabel('Errors')
    plt.ylabel('Average Interaction Time (s)')
    plt.title('Average Interaction Time for Each error')
    plt.show()



# Directory containing your 5 folders, each with 9 files
directory_path = '/home/jvp6149/Desktop/bagfiles_Pang/organizedTrial9pour'

plot_average_interaction_times(directory_path)
    
"""
# Directory containing your text files
directory_path = 'C:/Users/junru/Downloads'

# List of file names you want to process, including "standard_CP.txt" and others
file_names = ["CP_peg_1.txt", "CP_peg_3.txt", "CP_peg_5.txt", "CP_peg_7.txt", "CP_peg_9.txt"]

# List to store the DataFrames for each file
dataframes = []

for file_name in file_names:
    file_path = os.path.join(directory_path, file_name)
    df = extract_and_normalize_data(file_path)

    if df is not None:
        df['file_name'] = file_name  # Add a column to store the file name
        dataframes.append(df)

# Plot the relationship between 'field.poseStamped.pose.position.x' and 'normalized_seq' for each file

plt.figure(figsize=(10, 6))
for df in dataframes:
    plt.plot(df['normalized_seq'], df['field.poseStamped.pose.position.x'], label=df.iloc[0]['file_name'])

plt.xlabel('Normalized Sequence')
plt.ylabel('field.poseStamped.pose.position.x')
plt.legend()
plt.title('Relationship between Position X and Normalized Sequence')
plt.show()
# Plot the relationship between 'field.poseStamped.pose.position.y' and 'normalized_seq' for each file
plt.figure(figsize=(10, 6))
for df in dataframes:
    plt.plot(df['normalized_seq'], df['field.poseStamped.pose.position.y'], label=df.iloc[0]['file_name'])

plt.xlabel('Normalized Sequence (0-1)')
plt.ylabel('field.poseStamped.pose.position.y')
plt.legend()
plt.title('Relationship between Position Y and Normalized Sequence')
plt.show()

# Plot the relationship between 'field.poseStamped.pose.position.z' and 'normalized_seq' for each file
plt.figure(figsize=(10, 6))
for df in dataframes:
    plt.plot(df['normalized_seq'], df['field.poseStamped.pose.position.z'], label=df.iloc[0]['file_name'])

plt.xlabel('Normalized Sequence (0-1)')
plt.ylabel('field.poseStamped.pose.position.z')
plt.legend()
plt.title('Relationship between Position Z and Normalized Sequence')
"""
