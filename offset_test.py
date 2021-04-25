import serial
import time

dwm0 = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
dwm1 = serial.Serial(port='/dev/ttyACM1', baudrate=115200)

print('Connected to ' + dwm0.name + ' and ' + dwm1.name)

#Arduino = serial.Serial(port='/dev/ttyACM2', baudrate=9600)


dwm0.write('\r\r'.encode())
dwm1.write('\r\r'.encode())
time.sleep(1)
dwm0.write('lec\r'.encode())
dwm1.write('lec\r'.encode())
time.sleep(1)

def getAvg(list):
    sum = 0
    for item in list:
        sum += item
    return sum/len(list)

window0 = []
window1 = []
avgs0 = [0,0,0,0]
avgs1 = [0,0,0,0]
win_size = 20

while True:
    try:
        line0 = dwm0.readline()
        #print('dwm0: ',end='')
        #print(line0)
        if len(line0) == 37 or len(line0) == 38:
            dist0 = float(line0[-6:-2])
            if len(line0) == 38:
                dist0 = float(line0[-7:-2])
            if len(window0) < win_size:
                window0.append(dist0)
            else:
                window0.pop(0)
                window0.append(dist0)
                for i in range(4):
                    avgs0[i] = getAvg(window0[i*5:i*5+5])

            print('dwm0: ',end='')
            print('%.2f, %.2f, %.2f' % (avgs0[-1], getAvg(avgs0[2:]), getAvg(avgs0)))
            '''''
            Arduino.write(dist0.encode('utf-8'))
            rec_msg0 = Arduino.readline().decode('utf-8', 'backslashreplace')
            print('FROM ARDUINO: ', end='')
            print(rec_msg0)
            '''''
    except:
        print("dwm0 Error")
        break
    try:
        line1 = dwm1.readline()
        #print('dwm1: ',end='')
        #print(line1)
        if len(line1) == 37 or len(line1) == 38:
            dist1 = float(line1[-6:-2])
            if len(line1) == 38:
                dist1 = float(line1[-7:-2])
            if len(window1) < win_size:
                window1.append(dist1)
            else:
                window1.pop(0)
                window1.append(dist1)
                for i in range(4):
                    avgs1[i] = getAvg(window1[i*5:i*5+5])

            print('dwm1: ',end='')
            print('%.2f, %.2f, %.2f' % (avgs1[-1], getAvg(avgs1[2:]), getAvg(avgs1)))
    except:
        print("dwm1 Error")
        break

dwm0.write('\r'.encode())
dwm1.write('\r'.encode())
dwm0.close()
dwm1.close()
