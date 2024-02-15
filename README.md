# HumanCorrectionsTasks
# Plot Interaction time
# Premise: 
Organize the folder for each task, for example "Pour". There should be 5 subfolders in task folder named by error types. In each subfolder, there are txt files recorded fromeach trial and each participants.
# Notice:
In each subfolder, there are also have a txt file which contains the "correct" trajectory without any human interaction. This "correct" file is averaged from 20 repeated no interaction txt files. For every task, there should be 5 "correct" trajectories since there are 5 error types. 
# Use
1. Chnage the folder path to the path of your task folder.
2. Plot
   `python3 plot_interaction.py`
3. Output to csv
   `python3 correct_clculation.py`
4. Get "Correct" trajectory
   `python3 correct_clculation.py`
