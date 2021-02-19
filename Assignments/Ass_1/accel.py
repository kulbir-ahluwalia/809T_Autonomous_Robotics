import numpy as np
import matplotlib
import matplotlib.pyplot as plt

####################################################################
# Load 5th entry of each row data into numpy array
# 5th entry = Pitch angle in degrees
####################################################################

readings = open('imudata.txt','r')
lines = readings.readlines()
# print(lines)

pitch_angles_list = []
row_number_list = []

line_number = 0
for line in lines:
	# print(line)

	# remove spaces at start and end of line
	line.strip()

	# get 5th reading
	pitch_angle = int(line.split(' ')[4])
	print(pitch_angle)

	pitch_angles_list.append(pitch_angle)
	row_number_list.append(line_number)


	line_number += 1

print(pitch_angles_list)
print(row_number_list)

readings.close()


####################################################################
# Plotting data
####################################################################
# plot pitch angle on y axis and row number on x axis with point marker (.)
# solid style line (-) and yellow colour (y)
plt.plot(row_number_list, pitch_angles_list,'.-y')

plt.show()











