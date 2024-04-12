import numpy as np

import os

import matplotlib.pyplot as plt
 
# Define dimensions of the KUKA LBR iiwa14 robot in millimeters

link_lengths = [360, 210, 210, 200, 200, 126, 126]  # Link lengths in millimeters
 
# Function to identify interaction start and stop points

def find_interaction_points(file_path, standard_torques, standard_seq):

    # Read data from file

    data = np.genfromtxt(file_path, delimiter=',', skip_header=1)
 
    # Extract relevant data

    seq = data[:, 0]

    torques = data[:, 4:11]  # Assuming torques are from a1 to a7
 
    interaction_start_points = []

    interaction_stop_points = []
 
    for i in range(len(torques)):

        # Calculate absolute difference between torques and standard torques

        diff = np.abs(torques[i] - standard_torques[i])
 
        # Check if difference exceeds thresholds for interaction start and stop

        if np.any(diff > 0.05):

            interaction_start_points.append(i)

        elif np.all(diff < 0.05) and len(interaction_start_points) > 0:

            interaction_stop_points.append(i)
 
    # If interaction stop point not found after start point, assume last sequence number as stop point

    if len(interaction_start_points) > len(interaction_stop_points):

        interaction_stop_points.append(len(standard_seq) - 1)
 
    return interaction_start_points, interaction_stop_points
 
# Function to calculate total interaction torque difference for joints 4, 5, and 6

def calculate_total_torque_difference(file_path, standard_torques, standard_seq):

    # Find interaction points

    interaction_start_points, interaction_stop_points = find_interaction_points(file_path, standard_torques, standard_seq)
 
    # Read data from file

    data = np.genfromtxt(file_path, delimiter=',', skip_header=1)
 
    # Extract relevant data

    seq = data[:, 0]

    torques = data[:, 4:11]  # Assuming torques are from a1 to a7
 
    # Calculate total torque difference for joints 4, 5, and 6

    total_torque_diff = np.zeros(3)

    for start, stop in zip(interaction_start_points, interaction_stop_points):

        diff = np.abs(np.sum(torques[start:stop + 1, 3:6], axis=0) - np.sum(standard_torques[start:stop + 1, 3:6], axis=0))

        total_torque_diff += 5*diff
 
    return total_torque_diff
 
# Function to calculate average interaction force from multiple files in a folder

# Function to calculate Jacobian matrix
def calculate_jacobian(theta, link_lengths):
    l1, l2, l3, l4, l5, l6 = link_lengths
    
    # Joint angles (convert to radians)
    theta1, theta2, theta3, theta4, theta5, theta6, theta7 = np.radians(theta)

    # Jacobian matrix components
    J11 = -l2 * np.sin(theta2) - l3 * np.sin(theta2 + theta3) - l4 * np.sin(theta2 + theta3 + theta4) - l5 * np.sin(theta2 + theta3 + theta4 + theta5) - l6 * np.sin(theta2 + theta3 + theta4 + theta5 + theta6) - l1 * np.sin(theta2 + theta3 + theta4 + theta5 + theta6 + theta1)
    J12 = -l3 * np.sin(theta2 + theta3) - l4 * np.sin(theta2 + theta3 + theta4) - l5 * np.sin(theta2 + theta3 + theta4 + theta5) - l6 * np.sin(theta2 + theta3 + theta4 + theta5 + theta6) - l2 * np.sin(theta2)
    J13 = -l4 * np.sin(theta2 + theta3 + theta4) - l5 * np.sin(theta2 + theta3 + theta4 + theta5) - l6 * np.sin(theta2 + theta3 + theta4 + theta5 + theta6) - l3 * np.sin(theta2 + theta3)
    J14 = -l5 * np.sin(theta2 + theta3 + theta4 + theta5) - l6 * np.sin(theta2 + theta3 + theta4 + theta5 + theta6) - l4 * np.sin(theta2 + theta3 + theta4)
    J15 = -l6 * np.sin(theta2 + theta3 + theta4 + theta5 + theta6) - l5 * np.sin(theta2 + theta3 + theta4 + theta5)
    J16 = -l6 * np.sin(theta2 + theta3 + theta4 + theta5 + theta6)
    J17 = 0

    J21 = l2 * np.cos(theta2) + l3 * np.cos(theta2 + theta3) + l4 * np.cos(theta2 + theta3 + theta4) + l5 * np.cos(theta2 + theta3 + theta4 + theta5) + l6 * np.cos(theta2 + theta3 + theta4 + theta5 + theta6) + l1 * np.cos(theta2 + theta3 + theta4 + theta5 + theta6 + theta1)
    J22 = l3 * np.cos(theta2 + theta3) + l4 * np.cos(theta2 + theta3 + theta4) + l5 * np.cos(theta2 + theta3 + theta4 + theta5) + l6 * np.cos(theta2 + theta3 + theta4 + theta5 + theta6) + l2 * np.cos(theta2)
    J23 = l4 * np.cos(theta2 + theta3 + theta4) + l5 * np.cos(theta2 + theta3 + theta4 + theta5) + l6 * np.cos(theta2 + theta3 + theta4 + theta5 + theta6) + l3 * np.cos(theta2 + theta3)
    J24 = l5 * np.cos(theta2 + theta3 + theta4 + theta5) + l6 * np.cos(theta2 + theta3 + theta4 + theta5 + theta6) + l4 * np.cos(theta2 + theta3 + theta4)
    J25 = l6 * np.cos(theta2 + theta3 + theta4 + theta5 + theta6) + l5 * np.cos(theta2 + theta3 + theta4 + theta5)
    J26 = l6 * np.cos(theta2 + theta3 + theta4 + theta5 + theta6)
    J27 = 0

    J31 = 0
    J32 = 0
    J33 = 0
    J34 = 0
    J35 = 0
    J36 = 0
    J37 = 1

    J = np.array([[J11, J12, J13, J14, J15, J16, J17],
                  [J21, J22, J23, J24, J25, J26, J27],
                  [J31, J32, J33, J34, J35, J36, J37]])
    
    return J

def calculate_average_interaction_force(folder_path):

    total_force = 0

    num_files = 0
 
    # Read standard torques from standard_JT.txt

    standard_file_path = os.path.join(folder_path, "standard.txt")

    standard_data = np.genfromtxt(standard_file_path, delimiter=',', skip_header=1)

    standard_seq = standard_data[:, 0]

    standard_torques = standard_data[:, 4:11]  # Assuming torques are from a1 to a7
 
    # Iterate over files in the folder

    for filename in os.listdir(folder_path):

        if filename.endswith(".txt") and filename != "standard.txt":

            file_path = os.path.join(folder_path, filename)
 
            # Calculate total torque difference for joints 4, 5, and 6

            total_torque_diff = calculate_total_torque_difference(file_path, standard_torques, standard_seq)
 
            # Convert torque difference to force difference along y-axis for each joint

            total_force += np.sum(total_torque_diff / link_lengths[3:6]*0.001)  # Add converted forces for joints 4, 5, and 6

            num_files += 1
 
    # Calculate average force

    if num_files > 0:

        average_force = total_force / num_files 

        return average_force

    else:

        return None
 
# Directory containing folders

main_directory = "/Users/pang/Desktop/PSU/HumanCorrection_copy/torque/draw_torque"  # Change this to your main directory path
 
# Initialize lists to store average interaction forces for each folder

average_interaction_forces = []

folder_names = []
 
# Iterate over folders in the main directory

for folder_name in os.listdir(main_directory):

    folder_path = os.path.join(main_directory, folder_name)

    if os.path.isdir(folder_path):

        folder_names.append(folder_name)

        # Calculate average interaction force for the folder

        average_interaction_force = calculate_average_interaction_force(folder_path)

        if average_interaction_force is not None:

            average_interaction_forces.append(average_interaction_force)
 
# Convert list of average interaction forces to numpy array

average_interaction_forces = np.array(average_interaction_forces)
 
# Plot bar plot

fig, ax = plt.subplots()

labels = folder_names

x = np.arange(len(labels))

ax.bar(x, average_interaction_forces, tick_label=labels)

ax.set_xlabel('Error type')

ax.set_ylabel('Total Interaction Force N')

ax.set_title('Total Interaction Force for Draw Task')

plt.xticks(rotation=45)

plt.show()
