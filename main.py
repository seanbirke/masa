import serial
import time

DWM = serial.Serial(port='/dev/ttyACM1', baudrate=115200)
Arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600)

print('Connected to ' + DWM.name + ' and ' + Arduino.name)

DWM.write('\r\r'.encode())
time.sleep(1)
DWM.write('lec\r'.encode())
time.sleep(1)

while True:
    try:
        line = DWM.readline()
        #print(line)
        if len(line) == 37 or len(line) == 38:
            dist = str(line[-6:-5] + line[-4:-2]) + '\n'
            if len(line) == 38:
                dist = str(line[-7:-5] + line[-4:-2]) + '\n'
            print(dist[2:-2])
            Arduino.write(dist.encode('utf-8'))
            #rec_msg = Arduino.readline().decode('utf-8', 'backslashreplace')
            #print("FROM ARDUINO: ", end='')
            #print(rec_msg)
    except:
        print("Line Read Error")
        break

DWM.write('\r'.encode())
DWM.close()
