
#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

const float pi = 3.14159;
const float L1 = 6; // the length of the first arm in cm
const float L2 = 4; // the length of the second arm in cm
const float L3 = 3; // the length of the third arm in cm
float x; // the first coordinate
float y; // the second coordinate
float q1; // the angle of the first motor in radian
float Q1; // the angle of the first motor in degree
float q2; // the angle of the second motor in radian
float Q2; // the angle of the second motor in degree
float q3; // the angle of the third motor in radian
float Q3; // the angle of the third motor in degree
float q ; // the sum of q1+q2+q3 in radian ( in inverse kinematic it's known )
float Q ; // the sum of Q1+Q2+Q3 in degree ( in inverse kinematic it's known )
const int button1pin = 2;
const int button2pin = 3;
int button1state = LOW; // first button is for forward kinematics
int button2state = LOW; // second button is for backward kinematics
float xa ;
float xb ;
float xc ;
float ya ;
float yb ;
float yc ;
float k1 ;
float k2 ;
float x2 ;
float y2 ;
float x2p;
float y2p;
float L1p;
float L2p; // all these variables will help us simplify long equations 


void setup() {
  
servo1.attach(5);
servo2.attach(6);
servo3.attach(7);
pinMode (button1pin ,INPUT);
pinMode (button2pin ,INPUT);

}

void loop() {
  

button1state = digitalRead(button1pin);
button2state = digitalRead(button2pin);

if (button1state == HIGH) // if button 1 is pressed , assume forward kinematics , the known variables are : Q1,Q2,Q3 
{
  Q1 = 10; // assume first angle 40 degree
  Q2 = 90; // assume second anagle 60 degree
  Q3 = 110; // assume third angle 110 degree , these values are assumptions now, later they will be inputted by the user
  Q = Q1 + Q2 + Q3 ; // the sum of all angles
  q1 = Q1*pi/ 180 ;// to convert q1 to radian , q1 = 0.698 
  q2 = Q2*pi/ 180 ;// to convert q2 to radian , q2 = 1.047
  q3 = Q3*pi/180 ; // to convert q2 to radian , q3 = 1.396
  q = Q*pi/180 ; 
  xa = L1*cos(q1);
  xb = L2*cos(q1+q2);
  xc = L3*cos(q1+q2+q3);
  x = xa + xb + xc; // the expected value of x , x = 0.9016
  ya = L1*sin(q1);
  yb = L2*sin(q1+q2);
  yc = L3*sin(q1+q2+q3);
  y = ya + yb + yc;  // the expected value of y , y = 7.795
  servo1.write(Q1); 
  servo2.write(Q2);
  servo3.write(Q3); // command to all motors to rotate with Q1,Q2,Q3 angles
  
  
  
}

if (button2state == HIGH) // if button 2 pressed assume inverse kinematics , the known variables are : X,Y,Q
{
  x = 6; 
  y = 7; // assumed values of x and y for the desired end effector position , later these values will be inputted by the user
  Q = 120; // assume Q which is the sum of Q1+Q2+Q3
  q = Q*pi/180 ; // Q in radian
  x2 = x - (L3*cos(q)) ; // x2 = 7.5
  y2 = y - (L3*sin(q)) ; // y2 = 4.4
  x2p = pow(x2,2); //x2^2
  y2p = pow(y2,2); //y2^2
  L1p = pow(L1,2); // L1^2
  L2p = pow(L2,2); //L2^2
  q2 = acos((x2p+y2p-(L1p+L2p))/(2*L1*L2)); // the same equation apply in both 2DOF and 3DOF , q2 = 1.056 in radian
  k1 = (L1+L2*cos(q2))*x2 ;
  k2 = (L2*y2*sin(q2)) ;
  q1 = acos((k1 + k2)/(x2p+y2p)); // q1 in radian = 0.1195
  q3 = q - (q1+q2) ; // q3 in radian = 0.918
  Q1 = q1*180/pi ; // Q1 in degree = 6.85
  Q2 = q2*180/pi; // Q2 in degree = 60.50
  Q3 = q3*180/pi; // Q3 in degree = 52.59 
  
  servo1.write(Q1); 
  servo2.write(Q2);
  servo3.write(Q3); //  the three servo motors will move with angles of Q1,Q2,Q3 so the end effector will reach x,y
  
}


}
