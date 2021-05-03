import numpy as np
import matplotlib.pyplot as plt

timeElapsedlist = [] #creating empty list
#opening the text file and extracting the 5th column
with open('time_elapsed_data_final.txt') as f:  #opening the text file
    for line in f:
        timeElapsedlist.append(float(line[0:15])) #extracting the first 15 digits of the float number for calculation
        
# # Graph 1
x = np.linspace(1,len(timeElapsedlist),len(timeElapsedlist)) #determining the x axis
y = timeElapsedlist  #determining the y axis
plt.figure(1)# initializing the first figure
plt.plot(x,y,'.-y',label='Raw data') #plotting
plt.title('Object Tracking: Processing Time') #assigning plot title
plt.xlabel('Frame number') #assigning x label
plt.ylabel('Processing Time (seconds)') #assigning y label
plt.legend(loc="upper right")#assigning legend and position of the legend
# plt.show() #showing the plot


# # Graph 2
plt.figure(2)# initializing the second figure
plt.title('Object Tracking: Processing Time') #assigning plot title
plt.xlabel('Processing Time (seconds)') #assigning x label
plt.ylabel('Frame') #assigning y label
x = timeElapsedlist#determining the x axis
num_bins = int((len(timeElapsedlist))/5) #The number of bars in the histogram can be tweaked here
print(num_bins)
# n, bins, patches = plt.hist(x, num_bins, facecolor='k', alpha=0.5) #plotting the histogram

n, bins, patches = plt.hist(x, num_bins, density =True) #plotting the histogram

plt.show() #showing the plot