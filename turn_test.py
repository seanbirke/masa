import serial
import time

Arduino = serial.Serial(port='/dev/ttyACM2', baudrate=9600, timeout=1)

uwb0 = '400' + '\n'
uwb1 = '400' + '\n'
new_ang = 170

isLeft = False

while True:
    try:
        old_ang = new_ang + 1

        if new_ang > old_ang:
            isLeft = not isLeft

        lr_bool = 1 # 0 for left, 1 for right
        if isLeft:
            lr_bool = 0
        lr_bool_s = str(lr_bool) + '\n'

        Arduino.write(uwb0.encode('utf-8'))
        Arduino.write(uwb1.encode('utf-8'))
        Arduino.write(lr_bool_s.encode('utf-8'))

        new_ang -= 2
        if new_ang == 0:
            new_ang = 170
        
        try:
            rec_msg = Arduino.readline().decode('utf-8', 'backslashreplace')
            print('FROM ARDUINO: ',end='')
            print(rec_msg)
        except:
            print('NO MSG REC')
            break


    except:
        print("Main Loop Error")
        break

Arduino.close()
