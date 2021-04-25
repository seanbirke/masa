import serial
import time

dwm0 = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
dwm1 = serial.Serial(port='/dev/ttyACM1', baudrate=115200)

print('Connected to ' + dwm0.name + ' and ' + dwm1.name)

Arduino = serial.Serial(port='/dev/ttyACM2', baudrate=9600)


dwm0.write('\r\r'.encode())
dwm1.write('\r\r'.encode())
time.sleep(1)
dwm0.write('lec\r'.encode())
dwm1.write('lec\r'.encode())
time.sleep(1)

while True:
    try:
        line0 = dwm0.readline()
        print('dwm0: ',end='')
        print(line0)
        if len(line0) == 37:
            dist0 = str(line0[-6:-5] + line0[-4:-2]) + '\n'
            print('dwm0: ',end='')
            print(dist0[2:-2])
            Arduino.write(dist0.encode('utf-8'))
            rec_msg0 = Arduino.readline().decode('utf-8', 'backslashreplace')
            print('FROM ARDUINO: ', end='')
            print(rec_msg0)
    except:
        print("dwm0 Error")
        break
    try:
        line1 = dwm1.readline()
        print('dwm1: ',end='')
        print(line1)
        if len(line1) == 37:
            dist1 = str(line1[-6:-5] + line1[-4:-2]) + '\n'
            print('dwm1: ',end='')
            print(dist1[2:-2])
    except:
        print("dwm1 Error")
        break

dwm0.write('\r'.encode())
dwm1.write('\r'.encode())
dwm0.close()
dwm1.close()
