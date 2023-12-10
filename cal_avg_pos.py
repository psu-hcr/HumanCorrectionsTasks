import os

def read_files_in_cp(cp_folder):
    data = []
    for file_name in os.listdir(cp_folder):
        if file_name.endswith(".txt"):
            file_path = os.path.join(cp_folder, file_name)
            with open(file_path, 'r') as file:
                content = file.readlines()
                data.extend([line.strip().split(",") for line in content[1:]])  # Skip header row

    return data

def read_text_files(directory):
    all_data = []
    for root, dirs, files in os.walk(directory):
        for subdir in dirs:
            cp_folder = os.path.join(root, subdir, 'CP')
            if os.path.exists(cp_folder) and os.path.isdir(cp_folder):
                print(f"Processing files in {cp_folder}")
                data = read_files_in_cp(cp_folder)
                all_data.extend(data)

    return all_data

# Replace 'path' with the actual path of the directory you want to start from
starting_directory = 'path'
data = read_text_files(starting_directory)

# 'data' contains the information from all text files

import csv

def normalize_time(data):
    seq_column = [int(row[1]) for row in data]  # Assuming 'field.poseStamped.header.seq' is at index 1
    print(f"done")
    min_seq, max_seq = min(seq_column), max(seq_column)
    print(f"done")

    normalized_time = [(int(row[1]) - min_seq) / (max_seq - min_seq) for row in data]

    return normalized_time

def calculate_average_positions(data):
    positions = [[float(row[4]), float(row[5]), float(row[6])] for row in data]  # Assuming x, y, z columns
    seq_column = [int(row[1]) for row in data]  # Assuming 'field.poseStamped.header.seq' is at index 1

    unique_seqs = set(seq_column)
    average_positions = []

    for seq_point in unique_seqs:
        seq_positions = [positions[i] for i in range(len(seq_column)) if seq_column[i] == seq_point]
        avg_position = [sum(pos) / len(pos) for pos in zip(*seq_positions)]
        average_positions.append([seq_point] + avg_position)

    return average_positions

def save_to_csv(cp_folder, normalized_time, average_positions):
    csv_filename = os.path.join(cp_folder, f"{os.path.basename(cp_folder)}_result.csv")
    with open(csv_filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['Normalized_Seq', 'Average_Position_X', 'Average_Position_Y', 'Average_Position_Z'])
        csv_writer.writerows(average_positions)

    print(f"Results saved to {csv_filename}")

# Process the data 
normalized_time = normalize_time(data_peg)
print(f"done")
average_positions = calculate_average_positions(data)
print(f"done")

# Replace 'cp_folder' with the actual path of the CP folder where the text files are located
cp_folder = 'save_path'
save_to_csv(cp_folder, normalized_time, average_positions)
