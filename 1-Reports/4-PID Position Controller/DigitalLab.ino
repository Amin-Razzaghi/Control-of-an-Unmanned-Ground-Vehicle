const int MVPin = A0;
const int outPin = 9;
int DV=500;
int minError = DV - 2700;
int maxError = DV;
int MV;
int MVD;
int error, errorLast;
float D = 0, I = 0;
float Ts = 0.01;
int u;
int Kp = 15, Kd = 0.1, Ki = 1;
void setup() {
  // put your setup code here, to run once:
  pinMode(MVPin, INPUT);
  pinMode(outPin, OUTPUT);
  Serial.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  MV = analogRead(MVPin);
  MVD = map(MV,0,1023,0,2700);
  errorLast = error;
  error = (DV - MVD);
  D = (error - errorLast)/Ts;
  I +=  error*Ts;
  if( I > 65535){
    I = 65535;
  }
  else if( I < -65535){
    I = -65535;
  }
  if(I * error < 0 ) {
    I = 0;
  }
 
  u = map((error*Kp) + (D*Kd) + (I*Ki), minError, maxError, 0, 255);  
  if(u > 255) 
    u = 255;
  else if(u < 0)
    u = 0; 
  analogWrite(outPin, u);
  delay(Ts * 1000);
  char str[50];
  sprintf(str,"error: %d\tu: %d",error, u);
  Serial.println(str);
}
