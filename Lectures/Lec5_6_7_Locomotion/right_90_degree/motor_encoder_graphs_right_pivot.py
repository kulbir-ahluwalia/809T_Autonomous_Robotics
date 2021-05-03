#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
from matplotlib import pyplot as plt


# In[2]:


all_gpio_val = []
# file1 = open("gpio_values.txt","r") 
file1 = open("BR_gpio_states.txt","r") 
# file1 = open("BR_gpio_states0.8m_forward.txt","r") 
content = file1.readlines()
for line in content: 
      
    for i in line: 
          
        # Checking for the digit in  
        # the string 
        if i.isdigit() == True: 
              
            all_gpio_val.append(int(i))
file1.close() 


# In[3]:


all_gpio_val_2 = []
# file1 = open("encoder_values.txt","r") 


file1 = open("FL_gpio_states.txt","r") 

# file1 = open("FL_gpio_states0.8_forward.txt","r") 
content = file1.readlines()
for line in content: 
      
    for i in line: 
          
        # Checking for the digit in  
        # the string 
        if i.isdigit() == True: 
              
            all_gpio_val_2.append(int(i))
file1.close()


# In[5]:


x = np.linspace(0,len(all_gpio_val),len(all_gpio_val)+1)
x = list(x)
x.pop()


# In[7]:


x2 = np.linspace(0,len(all_gpio_val_2),len(all_gpio_val_2)+1)
x2 = list(x2)
x2.pop()


# In[9]:

fig, (ax1, ax2) = plt.subplots(2)
fig.suptitle('Motor Encoder Analysis for right pivot')
ax1.plot(x,all_gpio_val,'g')
ax2.plot(x2,all_gpio_val_2,'b')

ax1.set(ylabel='Encoder State Back Right')
ax2.set(ylabel='Encoder State Front Left')


# plt.plot(x,all_gpio_val,'r')
# plt.plot(x,all_gpio_val_2,'b')


# plt.plot(x2,all_gpio_val_2,'b')
# plt.title('Motor Encoder Analysis')
plt.savefig('motor_encoder_states_graph.png')
plt.xlabel('GPIO Input Reading')
# plt.ylabel('Encoder State ')
plt.show()


# In[ ]:




