import serial 
import time
import math

# Connect to uwb modules on specified ports, must be plugged in in order
dwm0 = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)
dwm1 = serial.Serial(port='/dev/ttyACM1', baudrate=115200, timeout=0.1)

print('Connected to ' + dwm0.name + ' and ' + dwm1.name)

# Arduino is defined on ttyACM2 so must be plugged in third
Arduino = serial.Serial(port='/dev/ttyACM2', baudrate=9600, write_timeout=0.1)

# init uwb modules, start outputting distances
dwm0.write('\r\r'.encode())
dwm1.write('\r\r'.encode())
time.sleep(1)
dwm0.write('lec\r'.encode())
dwm1.write('lec\r'.encode())
time.sleep(1)
#dwm0.write('\r'.encode())
#dwm1.write('\r'.encode())
#time.sleep(1)

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

# distance between uwb on the suitcase [m]
# 9 inches = 0.2286m
c_dist = 0.2286

# error in uwb distance = +09 cm
offset = 0.09

# lr_bool: 0:Left, 1:Right, 2:Straight
lr_bool = 2

# difference threshold in uwb distance for command = straight
straight_thresh = 0.125


while True:
    try:
        # Read uwb distances as byte encoded lines
        line0 = dwm0.readline()
        line1 = dwm1.readline()
        
        print("line0: ",end='')
        print(line0)
        print("line1: ",end='')
        print(line1)

        # Extract distance from line, case on <1.00m == len 37 or >1.00m == len 38
        if len(line0) == 37:
            dist0_f = float(line0[-6:-2])
        elif len(line0) == 38:
            dist0_f = float(line0[-7:-2])

        if len(line1) == 37:
            dist1_f = float(line1[-6:-2])
        elif len(line1) == 38:
            dist1_f = float(line1[-7:-2])

        # Added offset for uwb error correction
        dist0_f += offset
        dist1_f += offset

        # Add to averaging window, that averages distances over 1s
        if len(window0) < win_size:
            window0.append(dist0_f)
        else:
            window0.pop(0)
            window0.append(dist0_f)
        if len(window1) < win_size:
            window1.append(dist1_f)
        else:
            window1.pop(0)
            window1.append(dist1_f)

        if len(window0) == win_size and len(window1) == win_size:
            # Distances averaged over win_size number of readings
            dist0_a = getAvg(window0)
            dist1_a = getAvg(window1)

            big = max(dist0_a, dist1_a)
            sm = min(dist0_a, dist1_a)

            # Cosine Rule for calculating heading, uwb placed sideways
            loc = (c_dist**2 + big**2 - sm**2)/(2*big*c_dist)
            if loc < -1:
                loc = -1
            elif loc > 1:
                loc = 1
            angle = 90 - math.degrees(math.acos(loc))
            print('angle: ',end='')
            print(angle)

            # lr_bool determines direction of turning
            if abs(dist0_a - dist1_a) < straight_thresh:
                lr_bool = 2
            elif dist0_a < dist1_a:
                lr_bool = 0
            elif dist1_a < dist0_a:
                lr_bool = 1

            dist0_s = str(round(dist0_f*100))
            if len(dist0_s) == 2:
                dist0_s = '0' + dist0_s

            dist1_s = str(round(dist1_f*100))
            if len(dist1_s) == 2:
                dist1_s = '0' + dist1_s

            lr_bool_s = str(lr_bool) + '\n'

            print("uwb0 dist: ", dist0_s)
            print("uwb1 dist: ", dist1_s)
            print("lr bool: ", lr_bool_s)

            # Write and Read from Arduino
            try:
                msg_to_arduino = dist0_s + dist1_s + lr_bool_s
                print(msg_to_arduino)
                Arduino.write(msg_to_arduino.encode())
                # Arduino.write(dist0_s.encode('utf-8'))
                # Arduino.write(dist1_s.encode('utf-8'))
                # Arduino.write(lr_bool_s.encode('utf-8'))
            except:
                print("Error sending to Arduino")
            '''''
            try:
                rec_msg = Arduino.readline().decode('utf-8', 'backslashreplace')
                print('From Arduino: ', end='')
                print(rec_msg)
            except:
                print("Error receiving from Arduino")
            '''''
    except:
        msg_to_arduino = "0100102" + '\n'
        Arduino.write(msg_to_arduino.encode())
        print("Stopping MASA")
        print("Error reading uwb lines or in mainloop")
        break

dwm0.write('\r'.encode())
dwm1.write('\r'.encode())
dwm0.close()
dwm1.close()


