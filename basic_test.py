import serial
import time
import math

Arduino = serial.Serial(port='/dev/ttyACM2', baudrate=9600, write_timeout=0.1)
dwm0 = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)
dwm1 = serial.Serial(port='/dev/ttyACM1', baudrate=115200, timeout=0.1)

print('Connected to ' + dwm0.name + ' and ' + dwm1.name)

dwm0.write('\r\r'.encode())
dwm1.write('\r\r'.encode())
time.sleep(1)
dwm0.write('lec\r'.encode())
dwm1.write('lec\r'.encode())
time.sleep(1)

while True:
    try:
        line0 = dwm0.readline()
        line1 = dwm1.readline()

        print('line0: ', end='')
        print(line0)
        print('line1: ', end='')
        print(line1)

        if len(line0) != 0:
            print("line0: ", end='')
            print(line0)
        if len(line1) != 0:
            print("line1: ", end='')
            print(line1)
        #dwm0.write('\r'.encode())
        #dwm1.write('\r'.encode())

        msg = "2002002\n"
        Arduino.write(msg.encode())
        print("sent")
        '''''
        rec_msg = Arduino.readline().decode('utf-8', 'backslashreplace')
        print('From Arduino: ', end='')
        print(rec_msg)
        '''''

    except:
        print("Read Error")
        dwm0.write('\r'.encode())
        dwm1.write('\r'.encode())
        dwm0.close()
        dwm1.close()
        break

