import serial 
import time
import math

# Connect to uwb modules on specified ports, must be plugged in in order
dwm0 = serial.Serial(port='/dev/tty.usbmodem0007601200521', baudrate=115200)
dwm1 = serial.Serial(port='/dev/tty.usbmodem0007601193921', baudrate=115200)

print('Connected to ' + dwm0.name + ' and ' + dwm1.name)

# init uwb modules, start outputign distances
dwm0.write('\r\r'.encode())
dwm1.write('\r\r'.encode())
time.sleep(1)
dwm0.write('lec\r'.encode())
dwm1.write('lec\r'.encode())
time.sleep(1)

# distances as floats
dist0_f = 0
dist1_f = 0

# distances as strings
dist0_s = ""
dist1_s = ""

# find avgerage value for a list
def getAvg(lst):
    if not lst:
        return 0
    sum = 0
    for item in lst:
        sum += item
    return sum/len(lst)

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

# lr_bool: 0:Left, 1:Right, 2:Straigt
lr_bool = 2

# difference threshold in uwb distance for command = straight (lr_bool = 2)
straightThres = 0.15

while True:
    try:
        # Read uwb distances as byte encoded lines
        line0 = dwm0.readline()
        line1 = dwm1.readline()

        #print(line0)
        #print(line1)

        # Extract distance from line, case on <1.00m == len 37 or >1.00m == len 38
        if len(line0) == 37:
            dist0_f = float(line0[-6:-2])
        elif len(line0) == 38:
            dist0_f = float(line0[-7:-2])
        else:
            print(len(line0))
            
        if len(line1) == 37:
            dist1_f = float(line1[-6:-2])
        elif len(line1) == 38:
            dist1_f = float(line1[-7:-2])
        else:
            print(len(line1))

        # Added offset for uwb error correction
        dist0_f += offset
        dist1_f += offset

        big = max(dist0_f, dist1_f)
        sm = min(dist1_f, dist1_f)

        # Cosine Rule for calculating heading, uwb placed sideways
        loc = (c_dist**2 + big**2 - sm**2)/(2*big*c_dist)
        if loc < -1:
            loc = -1
        elif loc > 1:
            loc = 1
        angle = 90 - math.degrees(math.acos(loc))

        # lr_bool determines direction of turning
        if abs(dist0_f - dist1_f) < 0.15:
            lr_bool = 2
        elif dist0_f < dist1_f:
            lr_bool = 0
        elif dist1_f < dist0_f:
            lr_bool = 1

        # print("UWB0 dist: ", dist0_f)
        # print("UWB1 dist: ", dist1_f)
        # print("L/R Boolean: ", lr_bool)

        dist0_s = str(round(dist0_f*100)) + '\n'
        dist1_s = str(round(dist1_f*100)) + '\n'
        lr_bool_s = str(lr_bool) + '\n'

        print("uwb0 dist: ", dist0_s)
        print("uwb1 dist: ", dist1_s)
        print("lr bool: ", lr_bool_s)

    except:
        print("Can't read UWB lines")
        break

dwm0.write('\r'.encode())
dwm1.write('\r'.encode())
dwm0.close()
dwm1.close()



