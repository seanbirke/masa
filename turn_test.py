import serial
import time

Arduino = serial.Serial(port='/dev/ttyACM2', baudrate=9600)

uwb0 = '400' + '\n'
uwb1 = '400' + '\n'
angle = 170

while True:
    try:
        angle_s = str(angle) + '\n'
        old_ang = angle - 1
        old_ang_s = str(old_angle) + '\n'

        Arduino.write(uwb0.encode('utf-8'))
        Arduino.write(uwb1.encode('utf-8'))
        Arduino.write(old_ang_s.encod('utf-8'))
        Arduino.write(angle_s.encode('utf-8'))

        rec_msg = Arduino.readline().decode('utf-8', 'backslashreplace')
        print('FROM ARDUINO: ',end='')
        print(rec_msg)

        angle -= 2
        if angle == 0:
            angle = 170

    except:
        print("Error Found")
        break

Arduino.close()
