import serial
import time

Arduino = serial.Serial(port='/dev/ttyACM2', baudrate=9600, timeout=1)

print("Connected to " + Arduino.name)

while True:
    try:
        #msg = "Hello from raspi" + '\n'
        #msg = 'dd90' + '\n'
        uwb0 = '400' + '\n'
        uwb1 = '300' + '\n'
        angle = '100' + '\n'
        lr_flag = '1' + '\n'
        Arduino.write(uwb0.encode('utf-8'))
        Arduino.write(uwb1.encode('utf-8'))
        Arduino.write(angle.encode('utf-8'))
        Arduino.write(lr_flag.encode('utf-8'))

        rec0 = Arduino.readline().decode('utf-8', 'backslashreplace')
        print(rec0)

    except:
        print("Error")
        break

Arduino.close()
