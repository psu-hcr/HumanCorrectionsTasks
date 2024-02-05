library(pwr)
library(snow)
library(doSNOW)

#sink data to text file output
sink("/home/qxa5031/iiwa_ws/src/haptics_controls/src/Plots/p_values.txt")
#import csv into r
p_data <- read.csv("/home/qxa5031/iiwa_ws/src/haptics_controls/src/Data/alldata.csv")
#attach the data to the script so we can use the headers to call data sets
df <- attach(p_data)
#assign our factors
subj <-factor(Participant Number)
error <-factor(Error Type)
int <- factor(Interaction Time)
trial <-factor(Trial Number)

print("Anova Results for the Angle of Motion")
#compute anova for angle measures

#model1 <- aov(abs_angle_error ~ (con*ang*mag) + Error(subj/(con*ang*mag)))
#print(summary(model1))

cl <- makeCluster(3, type = "SOCK")
registerDoSNOW(cl)
model_1 <- aov(abs_angle_error ~ (error*int*trial) + Error(subj/(error*int*trial)))
atest <- summary(model_1)
astructure <- str(atest)
print(atest)
registerDoSEQ()
stopCluster(cl)
