import serial
import time
import math

# Connect to uwb modules on specified ports, must be plugged in in order
dwm0 = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
dwm1 = serial.Serial(port='/dev/ttyACM1', baudrate=115200)

print('Connected to ' + dwm0.name + ' and ' + dwm1.name)

# Arduino is defined on ttyACM2 so must be plugged in third
Arduino = serial.Serial(port='/dev/ttyACM2', baudrate=9600)

# init uwb modules, start outputign distances
dwm0.write('\r\r'.encode())
dwm1.write('\r\r'.encode())
time.sleep(1)
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

# distance between uwb on the suitcase [m]
# 16 inches = 0.4064m
c_dist = 0.4064

# error in uwb distance = +09 cm
offset = 0.09

while True:
    try:
        # Read line from uwb
        line0 = dwm0.readline()
        line1 = dwm1.readline()
        # Extract distance from line, case on <10m or >10m (line len 37 or 38)
        if len(line0) == 37 or len(line0) == 38:
            #dist0s = str(line0[-6:-5] + line0[-4:-2]) + '\n'
            dist0f = float(line0[-6:-2])
            if len(line0) == 38:
                #dist0s = str(line0[-7:-5] + line0[-4:-2]) + '\n'
                dist0f = float(line0[-7:2])
            if len(window0) < win_size:
                window0.append(dist0f)
            else:
                window0.pop(0)
                window0.append(dist0f)
        if len(line1) == 37 or len(line1) == 38:
            #dist1s = str(line1[-6:-5] + line1[-4:-2]) + '\n'
            dist1f = float(line1[-6:-2])
            if len(line1) == 38:
                #dist1s = str(line1[-7:-5] + line1[-4:-2]) + '\n'
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

            loc = (c_dist**2 + dist0a**2 - dist1a**2)/(2*dist0a*c_dist)
            if loc < -1:
                loc = -1
            elif loc > 1:
                loc = 1
            angle = 180 - math.degrees(math.acos(loc))

            dist0s = str(int(dist0a*100)) + '\n'
            dist1s = str(int(dist1a*100)) + '\n'
            angle = str(round(angle)) + '\n'
            #'''''
            print("uwb0 dist: ", dist0s)
            print("uwb1 dist: ", dist1s)
            print("calculated angle: ", angle)
            #'''''
            Arduino.write(dist0s.encode('utf-8'))
            Arduino.write(dist1s.encode('utf-8'))
            Arduino.write(angle.encode('utf-8'))

            rec_msg = Arduino.readline().decode('utf-8', 'backslashreplace')
            print('FROM ARDUINO: ', end='')
            print(rec_msg)

    except:
        print("Unspecified Error")
        break

dwm0.write('\r'.encode())
dwm1.write('\r'.encode())
dwm0.close()
dwm1.close()
