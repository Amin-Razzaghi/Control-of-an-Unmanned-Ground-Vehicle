
#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define ABS(a) ((a<0)?(-a):(a))

#define JOYSTICK_DELAY 50000
#define MOTOR_SPEED_MAX 200
#define MOTOR_SPEED_STEP 17
#define MOTOR_SPEED_TURN 70
#define MOTOR_SPEED_MINISTEP 10

typedef struct u_time{
    unsigned long int sec;
    unsigned long int usec;
} U_Time;

typedef struct autRobot{
    int portfd;		// File descriptor of robot which data in written to and has to be read from

    int motorSpeedLeft;		// The number has to be set on the left motor input
    int motorSpeedRight;	// The number has to be set on the right motor input

    int motorShaftLeft;		// Left shaft encoder data
    int motorShaftRight;	// Right shaft encoder data

    double omegaL;		// Left angular velocity
    double omegaR;		// Right angular velocity

    int sonarRear;			// data of sonar number 0
    int sonarRearL;			// data of sonar number 1
    int sonarFrontL;			// data of sonar number 2
    int sonarFront;			// data of sonar number 3
    int sonarFrontR;			// data of sonar number 4
    int sonarRearR;			// data of sonar number 5

    U_Time updateTime;		// relative time of robot's update

    double battery;		// battery life percentage

    int reset;			// does robot need reseting?
} AUTRobot;

void changemode(int);
int  kbhit(void);
void joystickMode(AUTRobot *robot);

int connect(AUTRobot *robot);
int updateRobot(AUTRobot *robot);
void robotToString(AUTRobot *robot, char *dst);
int rotateRobot(AUTRobot *robot, float degree);
int setMotorSpeeds(AUTRobot *robot, int left, int right, int display);
int updateOmegaTable(AUTRobot *robot);

int main(int argc, char *argv[])
{
    AUTRobot a;
    AUTRobot *robot = &a;
    char robotINF[1024];
    int c, i, dms=10;
//TRY TO CONNECT TO ROBOT
    if (connect(robot) == -1){
        return -1;
    }
 printf("Connected: %d\n", robot->portfd);
    
    robot->reset = 1;
//set the speed of motor to zero
    setMotorSpeeds(robot, 0, 0, 1);
    
 
   joystickMode(robot);

    robotToString(robot, robotINF);
    printf("Data:\n %s", robotINF);
    close(robot->portfd);
    return 0;
};

//connects to the proper serial port and sets the file descriptor 
int connect(AUTRobot *robot)
{
    int fd;
//CONNECTING TO /dev/ttyUSB0 AS A SERIAL PORTWITH ABILITY OF READ AND WRITE ON IT
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    struct termios options;
    
    if (fd == -1)
        perror("Error(connect): Can not open Serial Port\n");
    else{
        fcntl(fd, F_SETFL, FNDELAY);
        tcgetattr(fd, &options);
//SETTING THE BAUDRATE OF SERIAL PORT FOR COMINUCATION
        cfsetispeed(&options, B38400);
        cfsetospeed(&options, B38400);
//SEETING SOME FLAG FOR THROW THE EXCEPTION
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        tcsetattr(fd, TCSANOW, &options);
//ASSIGN THE robot->portfd
        robot->portfd = fd;
    }
    return fd;
}
//this function write the command on serial port for running on robot
int updateRobot(AUTRobot *robot)
{
    int i, data[8];
    char command[20];
    char buffer[50];  /* Input buffer */
    char *bufptr;      /* Current char in buffer */
    int  nbytes;       /* Number of bytes read */
//if the serial port was closed do this function
    if(robot->portfd < 1)
    {
        perror("Error(updateRobot): Robot Serial Port is not open\n");
        return -1;
    }
//if the robot is reset do this function
    if(robot->reset == 1) {
//motor speed is set to be zero both side
        sprintf(command, "R%4d %4d\n", robot->motorSpeedRight, robot->motorSpeedLeft);
        robot->reset = 0;
    }
//upadet the right and left motor speed 
else {
        sprintf(command, "S%4d %4d\n", robot->motorSpeedRight, robot->motorSpeedLeft);
    }
//write the output command on serial port
    write(robot->portfd, command, 12);

       /* read characters into our string buffer until we get a CR or NL */
//read the input via serial port from robot
    for(i = 0; i < 9; i++)
    {
        bufptr = buffer;
        while (1)
        {
//read 1 byte frome serial port
            nbytes = read(robot->portfd, bufptr,1);
            if(nbytes > 0){
//append the read byte to output string
                bufptr += nbytes;
//checking the end of  each command 
                if (bufptr[-1] == '\n' || bufptr[-1] == '\r')
                    break;
            }
        }
        *bufptr = '\0';
//for command one to 8 do this function
        if (i < 8)
            data[i] = atoi(buffer);
//command 9 is for battery of robot and set this in to the  battery attribute of robot in structure
        else
            robot->battery = atof(buffer);
    }
//set this command to each attribute of robot in structure
    robot->motorShaftLeft = data[0];
    robot->motorShaftRight = data[1];

    robot->sonarRear = data[2];
    robot->sonarRearL = data[3];
    robot->sonarFrontL = data[4];
    robot->sonarFront = data[5];
    robot->sonarFrontR = data[6];
    robot->sonarRearR = data[7];

    return 0;
}
//printing the data of robot into the terminal 
void robotToString(AUTRobot *robot, char *dst)
{
    char pfd[70], mspl[70], mspr[70], mshl[70], mshr[70], bt[70];
    char  sr[70], srl[70], sfl[70], sf[70], sfr[70], srr[70];
    
    sprintf(pfd, "Port File Descriptor: %d", robot->portfd);
    sprintf(mspl, "Left Motor Speed: %d", robot->motorSpeedLeft);
    sprintf(mspr, "Right Motor Speed: %d", robot->motorSpeedRight);
    sprintf(mshl, "Left Shaft Encoder Data: %d", robot->motorShaftLeft);
    sprintf(mshr, "Right Shaft Encoder Data: %d", robot->motorShaftRight);
    sprintf(sr, "Rear Sonar Data: %d", robot->sonarRear);
    sprintf(srl, "Rear-Left Sonar Data: %d", robot->sonarRearL);
    sprintf(sfl, "Front-Left Sonar Data: %d", robot->sonarFrontL);
    sprintf(sf, "Front Sonar Data: %d", robot->sonarFront);
    sprintf(sfr, "Front-Right Sonar Data: %d", robot->sonarFrontR);
    sprintf(srr, "Rear-Right Sonar Data: %d", robot->sonarRearR);
    sprintf(bt, "Battery Voltage: %f", robot->battery);

    sprintf(dst,"\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n", pfd, mspl, mspr, mshl, mshr, sr, srl, sfl, sf, sfr, srr, bt);
}
//this function is for the rotation of the robot from the set degree
int rotateRobot(AUTRobot *robot, float degree)
{
    int sign = (degree > 0)?(1):(-1);
    int i;

    robot->motorSpeedLeft = sign * (-50);
    robot->motorSpeedRight = sign * 50;
    robot->reset = 1;
    updateRobot(robot);
    usleep(sign * degree * 900000);
    robot->motorSpeedLeft = 0;
    robot->motorSpeedRight = 0;
    updateRobot(robot);
    return 0;
}
int moveRobot(AUTRobot *robot, float distance)
{
    int stat=0;
    return stat;
}
//this function is about  the movement of the robot controlling by keyboard
void joystickMode(AUTRobot *robot)
{
    int ch, left = 0, right = 0;
    char robotINF[1024];
    
    changemode(1);
    
    while(1)
    {
//kbhit is for checking if the keyboard was pressed
        if(kbhit())
        {
//defining which key is for which movement
            ch = getchar();
//if the ‘esc’ is pressed the programme after setting the speed to zero
            if (ch == 27)
            {
                left = right = 0;
                setMotorSpeeds(robot, left, right, 1);
                usleep(JOYSTICK_DELAY);
                break;
            }
//if the W key pressed the upper case or lower case move forward(for moving forward you should set both left and right motor speed the same and lower than the Max_value)
            switch(ch)
            {
                case 'W':
                case 'w':
                    if (left < MOTOR_SPEED_MAX || right < MOTOR_SPEED_MAX){
                        left = MIN(MOTOR_SPEED_MAX, left + MOTOR_SPEED_STEP);
                        right = MIN(MOTOR_SPEED_MAX, right + MOTOR_SPEED_STEP);
                        setMotorSpeeds(robot, left, right, 1);
                    } else {
                        
                    }
                    break;
                   //if the S key pressed the upper or lower case move backward by setting the motors at mines speed of the forward move and lower than the Max_value
                case 'S':
                case 's':
                    if (left > -1*MOTOR_SPEED_MAX || right > -1*MOTOR_SPEED_MAX){
                        left = MAX(-1*MOTOR_SPEED_MAX, left - MOTOR_SPEED_STEP);
                        right = MAX(-1*MOTOR_SPEED_MAX, right - MOTOR_SPEED_STEP);
                        setMotorSpeeds(robot, left, right, 1);
                    } else {
                        
                    }
                    break;
                    
             //if the a key is pressed move to left by setting the right motor speed as high and the left motor speed low and lower than the Max_value   
                case 'A':
                case 'a':
                    if (left > -1*MOTOR_SPEED_MAX || right < MOTOR_SPEED_MAX){
                        left = MAX(-1*MOTOR_SPEED_MAX, left - MOTOR_SPEED_STEP);
                        right = MIN(MOTOR_SPEED_MAX, right + MOTOR_SPEED_STEP);
                        setMotorSpeeds(robot, left, right, 1);
                    } else {
                        
                    }
                    break;
                    
                 //if the d key is pressed   the lower case or upper case set the right motor speed as low and the left motor speed as high  and lower than the Max_value
                case 'D':
                case 'd':
                    if (left < MOTOR_SPEED_MAX || right > -1*MOTOR_SPEED_MAX){
                        left = MIN(MOTOR_SPEED_MAX, left + MOTOR_SPEED_STEP);
                        right = MAX(-1*MOTOR_SPEED_MAX, right - MOTOR_SPEED_STEP);
                        setMotorSpeeds(robot, left, right, 1);
                    } else {
                        
                    }
                    break;
                    
              //if the q key is pressed the upper case or lower case the robot move to north west       
                case 'Q':
                case 'q':
                    if (left != MOTOR_SPEED_TURN || right < MOTOR_SPEED_MAX){
                        if (left < MOTOR_SPEED_TURN)
                            left = MIN(MOTOR_SPEED_TURN, left + MOTOR_SPEED_MINISTEP);
                        else if (left > MOTOR_SPEED_TURN)
                            left = MAX(MOTOR_SPEED_TURN, left - MOTOR_SPEED_MINISTEP);
                        right = MIN(MOTOR_SPEED_MAX, right + MOTOR_SPEED_STEP);
                        setMotorSpeeds(robot, left, right, 1);
                    } else {
                        
                    }
                    break; 
                    
                    //if the e key is pressed the upper case or lower case the robot  move to north east
                case 'E':
                case 'e':
                    if (left < MOTOR_SPEED_MAX || right != MOTOR_SPEED_TURN){
                        left = MIN(MOTOR_SPEED_MAX, left + MOTOR_SPEED_STEP);
                        if (right < MOTOR_SPEED_TURN)
                            right = MIN(MOTOR_SPEED_TURN, right + MOTOR_SPEED_MINISTEP);
                        else if (right > MOTOR_SPEED_TURN)
                            right = MAX(MOTOR_SPEED_TURN, right - MOTOR_SPEED_MINISTEP);
                        setMotorSpeeds(robot, left, right, 1);
                    } else {
                        
                    }
                    break;
                    //if no key is pressed stop
                default:
                    left = right = 0;
                    setMotorSpeeds(robot, left, right, 1);
                    break;

            }

            while(kbhit())
                getchar();
            usleep(JOYSTICK_DELAY);
        } 
        else 
        {
            left = right = 0;
            setMotorSpeeds(robot, left, right, 1);
            usleep(JOYSTICK_DELAY);
        }
    }
    
    changemode(2);
}

// assign the motor speed to speed attribute of robot 
int setMotorSpeeds(AUTRobot *robot, int left, int right, int display)
{
    robot->motorSpeedRight = right;
    robot->motorSpeedLeft = left;
    
    updateRobot(robot);
//print the motor speed on terminal
    if (display)
    {
        char robotINF[1024];
        robotToString(robot, robotINF);
        printf("%s", robotINF); 
    }
    return 0;
}

void changemode(int dir)
{
    static struct termios oldt, newt;
    
    if ( dir == 1 )
    {
        tcgetattr( STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~( ICANON | ECHO );
        tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    }
    else
        tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}

int kbhit (void)
{
    struct timeval tv;
    fd_set rdfs;
    
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    
    FD_ZERO(&rdfs);
    FD_SET (STDIN_FILENO, &rdfs);
    
    select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);
    
}

