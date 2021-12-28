import serial
import readchar
import time
import getch
import keyboard

maxSpeed = 200
minSpeed = -200
motorSpeedTurn = 70
motorSpeedStep = 17
motorSpeedStepMin = 10



rightSpeed = 0
leftSpeed = 0


ser = serial.Serial('/dev/ttyUSB0')  # open serial port
ser.baudrate = 38400  #set baudrate TO 38400 
print(ser.name)         # check which port was really used
c = ''   
print("hellooooo")
s = ""
while (1):	    
    if getch.getch():          
        c = getch.getch()         #if the W key pressed the upper case or lower case move forward(for moving forward you should set both left and right motor speed the same and lower than the Max_value)

        if c == 'w' or c == 'W': #
            if rightSpeed < maxSpeed or leftSpeed < maxSpeed:
                rightSpeed = min( rightSpeed + motorSpeedStep , maxSpeed)
                leftSpeed = min( leftSpeed + motorSpeedStep , maxSpeed )
            s = 'S' + str(rightSpeed) + ' ' + str(leftSpeed) + '\n'
            ser.write(s.encode())
        # ser.write('S%d %d\n',(rightSpeed, leftSpeed))

        elif c == 'a' or c == 'A':                 #if the a key is pressed move to left by setting the right motor speed as high and the left motor speed low and lower than the Max_value   

            if rightSpeed < maxSpeed or leftSpeed > minSpeed:
                rightSpeed = min( rightSpeed + motorSpeedStep , maxSpeed)
                leftSpeed = max( leftSpeed - motorSpeedStep , minSpeed )
            s = 'S' + str(rightSpeed) + ' ' + str(leftSpeed) + '\n'
            ser.write(s.encode())
        # ser.write('S{} {}\n'.format(rightSpeed, leftSpeed))
                 #if the d key is pressed   the lower case or upper case set the right motor speed as low and the left motor speed as high  and lower than the Max_value
        elif c == 'd' or c == 'D':
            if rightSpeed > minSpeed or leftSpeed < maxSpeed:
                rightSpeed = max( rightSpeed - motorSpeedStep , minSpeed)
                leftSpeed = min( leftSpeed + motorSpeedStep , maxSpeed )
            s = 'S' + str(rightSpeed) + ' ' + str(leftSpeed) + '\n'
            ser.write(s.encode())
        # ser.write('S{} {}\n'.format(rightSpeed, leftSpeed))
                   #if the S key pressed the upper or lower case move backward by setting the motors at mines speed of the forward move and lower than the Max_value

        elif c == 's' or c == 'S':
            if rightSpeed > minSpeed or leftSpeed > minSpeed:
                rightSpeed = max( rightSpeed - motorSpeedStep , minSpeed)
                leftSpeed = max( leftSpeed - motorSpeedStep , minSpeed )
            s = 'S' + str(rightSpeed) + ' ' + str(leftSpeed) + '\n'
            ser.write(s.encode())
              #if the q key is pressed the upper case or lower case the robot move to north west       
        elif c == 'q' or c == 'Q':
            if rightSpeed < maxSpeed or leftSpeed != motorSpeedTurn:
                if (leftSpeed < motorSpeedTurn):
                    leftSpeed = min( leftSpeed + motorSpeedStepMin , motorSpeedTurn)
                elif ( leftSpeed > motorSpeedTurn):
                    leftSpeed = max( leftSpeed - motorSpeedStepMin , motorSpeedTurn)
                rightSpeed = min( rightSpeed + motorSpeedStep , maxSpeed)
            s = 'S' + str(rightSpeed) + ' ' + str(leftSpeed) + '\n'
            ser.write(s.encode())
                    #if the e key is pressed the upper case or lower case the robot  move to north east

        elif c == 'e' or c == 'E':
            if rightSpeed != motorSpeedTurn or leftSpeed < maxSpeed:
                if ( rightSpeed < motorSpeedTurn):
                    rightSpeed = min( rightSpeed + motorSpeedStepMin , motorSpeedTurn)
                elif ( rightSpeed > motorSpeedTurn):
                    rightSpeed = max( rightSpeed - motorSpeedStepMin , motorSpeedTurn)
                lefttSpeed = min( leftSpeed + motorSpeedStep , maxSpeed)
            s = 'S' + str(rightSpeed) + ' ' + str(leftSpeed) + '\n'
            ser.write(s.encode())
#if  key b is pressed it will stop (thi is used for emergency stop)
        elif c == 'b' :
            leftSpeed = 0
            rightSpeed = 0
            s = 'S' + str(rightSpeed) + ' ' + str(leftSpeed) + '\n'
            ser.write(s.encode())
            break;


    else:
        leftSpeed = 0
        rightSpeed = 0
        s = 'S' + str(rightSpeed) + ' ' + str(leftSpeed) + '\n'
        ser.write(s.encode())
#for printing the data that robot sent
    for i in range(9):
        print(ser.readline().decode())
    time.sleep(0.2);



ser.close()

