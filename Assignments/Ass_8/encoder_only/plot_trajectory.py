from matplotlib import pyplot as plt


plt.figure()
plt.title('Baron trajectory')
plt.xlabel('x axis (in m)')
plt.ylabel('y axis (in m)')
x= [0,1,1.02,0.05,0.03]
y= [0,0,0.81,0.78,0.01]
plt.grid()
plt.plot(x,y)
plt.savefig('robot_trajectory.jpg')