library(pwr)
library(snow)
library(doSNOW)

#sink data to text file output
sink("/home/zal5/iiwa_ws/src/haptics_controls/src/Plots/p_values.txt")
#import csv into r
p_data <- read.csv("/home/zal5/iiwa_ws/src/haptics_controls/src/Data/alldata.csv")
#attach the data to the script so we can use the headers to call data sets
df <- attach(p_data)
#assign our factors
subj <-factor(subject)
con <-factor(condition)
ang <- factor(desired_angle)
mag <-factor(desired_magnitude)

print("Anova Results for the Angle of Motion")
#compute anova for angle measures

#model1 <- aov(abs_angle_error ~ (con*ang*mag) + Error(subj/(con*ang*mag)))
#print(summary(model1))

cl <- makeCluster(2, type = "SOCK")
registerDoSNOW(cl)
model1 <- aov(abs_angle_error ~ (con*ang*mag) + Error(subj/(con*ang*mag)))
atest <- summary(model1)
astructure <- str(atest)
print(atest)
registerDoSEQ()
stopCluster(cl)

print('\n')
print("Anova Results for the Magnitude of Motion")
#compute anova for the speed measures

model2 <- aov(abs_rad_mag_error ~ (con*ang*mag) + Error(subj/(con*ang*mag)))
mtest <- summary(model2)
mstructure <- str(mtest)
print(mtest)
