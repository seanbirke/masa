#!/usr/bin/env/python3

import serial
import time
import math

# Connect to uwb modules on specified ports, must be plugged in in order
dwm0 = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
dwm1 = serial.Serial(port='/dev/ttyACM1', baudrate=115200)

print('Connected to ' + dwm0.name + ' and ' + dwm1.name)

# Arduino is defined on ttyACM2 so must be plugged in third
Arduino = serial.Serial(port='/dev/ttyACM2', baudrate=9600, timeout=1)

# init uwb modules, start outputign distances
dwm0.write('\r'.encode())
#time.sleep(1)
dwm1.write('\r'.encode())
#time.sleep(1)
dwm0.write('lec\r'.encode())
dwm1.write('lec\r'.encode())
time.sleep(1)

# find avgerage value for a list
def getAvg(list):
    sum = 0
    for item in list:
        sum += item
    return sum/len(list)

# stores distances readings
window0, window1 = [], []
# size of the above windows
# 10 Hz, so win_size=5 => 0.5 second window
win_size = 5

# holds previous angles
angles = []
# angle window size, first 5 old, next 5 new
num_angles = 10

# distance between uwb on the suitcase [m]
# 9 inches = 0.2286m
c_dist = 0.2286

# error in uwb distance = +09 cm
offset = 0.09

isLeft = True

# threshold in uwb distance differences, to determine straight bool
straight_thresh = 0.15

while True:
    
    try:
        # Read line from uwb
        #try:            
         #   line0 = dwm0.readline()
          #  line1 = dwm1.readline()
        #except:
         #   line0 = 'b'
          #  line1 = 'b'
        
        #while not lecFound0 and not lecFound1:
            #line0 = dwm0.readline()
            #line1 = dwm1.readline()
            #if len(line0) < 10 and not lecFound0:
            #    lecFound0 = True
            #if len(line1) < 10 and not lecFound1:
            #    lecFound1 = True
            #elif lecFound0 and lecFound1:
            #    break
            
        line0 = dwm0.readline()
        #print(line0)
        line1 = dwm1.readline()
        #print(line1)
        #print(len(line1))

        # Extract distance from line, case on <10m or >10m (line len 37 or 38)
        if len(line0) == 37 or len(line0) == 38:
            dist0f = float(line0[-6:-2])
            if len(line0) == 38:
                dist0f = float(line0[-7:2])
            if len(window0) < win_size:
                window0.append(dist0f)
            else:
                window0.pop(0)
                window0.append(dist0f)
        if len(line1) == 37 or len(line1) == 38:
            dist1f = float(line1[-6:-2])
            if len(line1) == 38:
                dist1f = float(line1[-7:2])
            if len(window1) < win_size:
                window1.append(dist1f)
            else:
                window1.pop(0)
                window1.append(dist1f)

        if len(window0) == win_size and len(window1) == win_size:
            # Distance values averaged over win_size number of readings
            dist0a = getAvg(window0) + offset
            dist1a = getAvg(window1) + offset

            big = max(dist0a, dist1a)
            sm = min(dist0a, dist1a)

            loc = (c_dist**2 + big**2 - sm**2)/(2*big*c_dist)
            if loc < -1:
                loc = -1
            elif loc > 1:
                loc = 1
            angle = 90 - math.degrees(math.acos(loc))

            if dist0a > 1 and dist1a > 1:
                if len(angles) >= num_angles:
                    angles.pop(0)
                angles.append(angle)
            
            if len(angles) == 1:
                old_ang = 0
            else:
                old_ang = getAvg(angles[0:len(angles)//2])
            new_ang = getAvg(angles[len(angles)//2:])

            #print("old_ang: ", old_ang)
            #print("new_ang: ", new_ang)
            
            if abs(dist0a - dist1a) < 0.15:
                lr_bool = 2
            elif dist0a < dist1a:
                lr_bool = 0
            elif dist1a < dist0a:
                lr_bool = 1


            dist0s = str(int(dist0a*100)) + '\n'
            dist1s = str(int(dist1a*100)) + '\n'
            lr_bool_s = str(lr_bool) + '\n'
            ##
            print("uwb0 dist: ", dist0s)
            print("uwb1 dist: ", dist1s)
            print("lr bool: ", lr_bool_s)
            ##
            Arduino.write(dist0s.encode('utf-8'))
            Arduino.write(dist1s.encode('utf-8'))
            Arduino.write(lr_bool_s.encode('utf-8'))
           
            try:
                print("start rec")
                rec_msg = Arduino.readline().decode('utf-8', 'backslashreplace')
                print('FROM ARDUINO: ', end='')
                print(rec_msg)
            except:
                print("NO REC MSG")
                break
            
    except:
        print("Unspecified Error")
        break
    #print(time.time() - t)


dwm0.write('\r'.encode())
dwm1.write('\r'.encode())
dwm0.close()
dwm1.close()
Arduino.close()
