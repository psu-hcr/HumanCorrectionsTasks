# HumanCorrectionsTasks
# Plot Interaction time
# Premise: 
Organize the folder for each task, for example, "Pour". There should be 5 subfolders in the task folder named by error types. Each subfolder contains txt files recorded fromeach trial and each participant.
# Notice:
In each subfolder, there is also a txt file that contains the "correct" trajectory without any human interaction. This "correct" file is averaged from 20 repeated no interaction txt files. For every task, there should be 5 "correct" trajectories since there are 5 error types. 
# Use
1. Change the folder path to the path of your task folder.
2. Plot
   `python3 plot_interaction.py`
3. Get "Correct" trajectory
   `python3 correct_clculation.py`

# JMP analysis
There is an example for data analysis using JMP
