import serial

#identify serial connections

ser = serial.Serial('/dev/ttyUSB0',9600)

count = 0

# new_x_angle = 0

while True:
    if(ser.in_waiting > 0):
        count+=1


        #read serial stream
        line = ser.readline()
        print(line)
        
        #avoid first n lines of serial info
        if(count>10):
            
            #strip serial stream of extra characters
            line = line.rstrip().lstrip()
            print(line)

            line = str(line)
            line = line.strip("'")
            line = line.strip("b'")
            print(line)


            #return float
            line = float(line)

            print(line,"\n")

































