import pandas as pd
import os
import matplotlib.pyplot as plt
import numpy as np
import csv

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

def process_text_file(file_path, sampling_rate=1/200):
    if not os.path.isfile(file_path):
        print(f"File not found: {file_path}")
        return None

    # Read the text file into a DataFrame
    df = pd.read_csv(file_path, sep=',')

    # Read the standard data (assuming the standard data is in a file named "standard_CP.txt" in the same directory)
    standard_file_path = os.path.join(os.path.dirname(file_path), "CP_peg_correct.txt")
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

# Directory containing your 5 folders, each with 9 files
participants = {
#1: "Trial4",
1: "Trial5",
2: "Trial6",
3: "Trial8",
4: "Trial9",
5: "Trial10",
6: "Trial11"
}

interaction_times = []  # Store interaction times for each file in the folder
interaction_data = []	#Store folder name and interaction data

for i in participants:
	directory_path = '/home/qxa5031/bagfiles_Quentin_copy/'+ participants[i] +'/peg/CP'
	#print(directory_path)
	
	folder_names = os.listdir(directory_path)
	average_interaction_times = []
	std_deviation_interaction_times = []

	for folder_name in folder_names:
		folder_path = os.path.join(directory_path, folder_name)
		file_names = os.listdir(folder_path)

		for file_name in file_names:
			singlefile = directory_path + file_name
			trial = singlefile[singlefile.rfind('P'):singlefile.rfind('.txt')]
			trial = trial[1:]
			participant_number = singlefile[singlefile.rfind('Trial'):singlefile.rfind('/p')]
			participant_number = participant_number[5:]
			file_path = os.path.join(folder_path, file_name)
			data, interaction_start_points, interaction_stop_points, total_interaction_time = process_text_file(file_path)
			interaction_times.append(total_interaction_time)
			foo = [participant_number, trial, folder_name, total_interaction_time]
			interaction_data.append(foo)
			#print(foo)
		#print(interaction_data)		

peg_close = []
peg_correct = []
peg_side = []
peg_sub = []
peg_wrong = []

for i in interaction_data:
	if i[2] == "peg_close":
		peg_close.append(i)            
	elif i[2] == "peg_correct":
		peg_correct.append(i)
	elif i[2] == "peg_side":
		peg_side.append(i)
	elif i[2] == "peg_sub":
		peg_sub.append(i)
	elif i[2] == "peg_wrong":
		peg_wrong.append(i)
					
all_data = [peg_close, peg_correct, peg_side, peg_sub, peg_wrong]
clean_data = []
avg_int_time = []
std_int_time = []

#Taking the standard file out of the data
for i in all_data:
	for j in i:	
		if j[1] != "_peg_correct":
			clean_data.append(j)

#print(clean_data)

#Writing each element of the clean_data list to a csv for ANOVA
with open('peg_data.csv', 'w', newline='') as file:
	writer = csv.writer(file)
	writer.writerow(['Participant', 'Trial', 'Error Type', 'Interaction Time'])	#Adding the labels to the first row
	for i in clean_data:		
		writer.writerow(i)	#Iterating through the clean data and adding each row to the csv

data_points = len(clean_data)
int_time = []

for i in clean_data:
	int_time.append(i[3])
	
	int_time_sum = sum(int_time)


# Calculate the average interaction time
avg_time = int_time_sum / data_points

#Calculate the standard deviation of interaction times
std_time = 0.2 * np.std(int_time)
avg_int_time.append(avg_time)
std_int_time.append(std_time)

"""
# Create a bar plot with error bars for average interaction times of the 5 folders
all_errors_labels = ["Close to Hole", "Correct", "Side Collision", "Sub-Optimal", "Wrong Hole"]
plt.figure(figsize=(10, 6))
plt.bar(all_errors_labels, avg_int_time, yerr=std_int_time, capsize=5)
plt.xlabel('Errors')
plt.ylabel('Average Interaction Time (s)')
plt.title('Average Interaction Time for Each error')
plt.show() 
"""
