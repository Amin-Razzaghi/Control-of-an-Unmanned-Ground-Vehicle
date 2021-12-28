import serial
import threading
from time import sleep

ser = serial.Serial('/dev/ttyUSB0')  # open serial port
ser.baudrate = 9600
print(ser.name)

ser2 = serial.Serial('/dev/ttyUSB1')
ser2.baudrate = 38400
print(ser2.name)         # check which port was really used


c = ''
flag = False
rightSpeed = 0
leftSpeed = 0


def read():
    global c, flag
    while True:
        c = ser.readline().decode()
        c = c[0]
        print(c)
        flag = True


def autRobot():
    maxSpeed = 200
    minSpeed = -200
    motorSpeedTurn = 70
    motorSpeedStep = 17
    motorSpeedStepMin = 10
    s = ""
    global flag, c, ser2, rightSpeed, leftSpeed, lock
    while (True):
        if flag:
            if c == 'W':
                if rightSpeed < maxSpeed or leftSpeed < maxSpeed:
                    rightSpeed = min(rightSpeed + motorSpeedStep, maxSpeed)
                    leftSpeed = min(leftSpeed + motorSpeedStep, maxSpeed)
            elif c == 'A':
                if rightSpeed < maxSpeed or leftSpeed > minSpeed:
                    rightSpeed = min(rightSpeed + motorSpeedStep, maxSpeed)
                    leftSpeed = max(leftSpeed - motorSpeedStep, minSpeed)
            elif c == 'D':
                if rightSpeed > minSpeed or leftSpeed < maxSpeed:
                    rightSpeed = max(rightSpeed - motorSpeedStep, minSpeed)
                    leftSpeed = min(leftSpeed + motorSpeedStep, maxSpeed)
            elif c == 'S':
                if rightSpeed > minSpeed or leftSpeed > minSpeed:
                    rightSpeed = max(rightSpeed - motorSpeedStep, minSpeed)
                    leftSpeed = max(leftSpeed - motorSpeedStep, minSpeed)
            elif c == 'Q':
                if rightSpeed < maxSpeed or leftSpeed != motorSpeedTurn:
                    if (leftSpeed < motorSpeedTurn):
                        leftSpeed = min(
                            leftSpeed + motorSpeedStepMin, motorSpeedTurn)
                    elif (leftSpeed > motorSpeedTurn):
                        leftSpeed = max(
                            leftSpeed - motorSpeedStepMin, motorSpeedTurn)
                    rightSpeed = min(rightSpeed + motorSpeedStep, maxSpeed)
            elif c == 'E':
                if rightSpeed != motorSpeedTurn or leftSpeed < maxSpeed:
                    if (rightSpeed < motorSpeedTurn):
                        rightSpeed = min(
                            rightSpeed + motorSpeedStepMin, motorSpeedTurn)
                    elif (rightSpeed > motorSpeedTurn):
                        rightSpeed = max(
                            rightSpeed - motorSpeedStepMin, motorSpeedTurn)
                    leftSpeed = min(leftSpeed + motorSpeedStep, maxSpeed)
            elif c == 'B':
                leftSpeed = 0
                rightSpeed = 0
            s = 'S' + str(rightSpeed) + ' ' + str(leftSpeed) + '\n'
            ser2.write(s.encode())
            flag = False

        else:
            pass
            # print('pass')
            # leftSpeed = 0
            # rightSpeed = 0
            # s = 'S' + str(rightSpeed) + ' ' + str(leftSpeed) + '\n'
            # ser2.write(s.encode())

        # for i in range(9):
        #     print(ser2.readline().decode())
        sleep(0.2)

    ser2.close()


if __name__ == '__main__':
    t1 = threading.Thread(target=autRobot)
    t2 = threading.Thread(target=read)
    t1.start()
    t2.start()
    t1.join()
    t2.join()
