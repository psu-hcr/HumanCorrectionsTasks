library(pwr)

#sink data to text file output
sink("/home/qxa5031/bagfiles_Quentin_copy/p_values_pour.txt")
#import csv into r
p_data <- read.csv("/home/qxa5031/bagfiles_Quentin_copy/pour_data.csv")
#attach the data to the script so we can use the headers to call data sets
df <- attach(p_data)
#assign our factors
subj <-factor(Participant)
error <-factor(Error_Type)
trial <-factor(Trial)

print("Anova Results for the Angle of Motion")
#compute anova for angle measures

#model1 <- aov(abs_angle_error ~ (con*ang*mag) + Error(subj/(con*ang*mag)))
#print(summary(model1))

model_1 <- aov(Interaction_Time ~ (error*trial) + Error(subj/(error*trial)))
atest <- summary(model_1)
astructure <- str(atest)
print(atest)
boxplot(Interaction_Time ~ (error * trial), data=, xlab="ErrorType/Trial", ylab = "Interaction_Time")
