//=========================================Introduction
/*Code written by Garrett Tjernagel for
  UND CEM EE480-481 SP22 Senior Design Capstone Project
  Augmented Robotic Manipulator (A.R.M)
  Partners: Branson Elliot and William Prody

*/

//=========================================Sources, Inspiration, and Links
/*


*/
//=========================================Libraries
//Gyro Stuff
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
//Radio Stuff
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//Servo Stuff
#include <Servo.h>

//=========================================Servo Things
//Arm Servos
Servo shoulder1Servo;
Servo shoulder2Servo;
Servo elbowServo;
Servo forearmServo;
Servo wristServo;

//Hand Servos
Servo thumbServo;
Servo pointerFingerServo;
Servo middleFingerServo;
Servo ringFingerServo;
Servo pinkyFingerServo;

//Servo Pins
int shoulder1Pin=0;
int shoulder2Pin=0;
int elbowPin=0;
int forearmPin=0;
int wristPin=0;

int thumbPin=0;
int pointerFingerPin=0;
int middleFinerPin=0;
int ringFingerPin=0;
int pinkyFingerPin=0;


//=========================================  2.Radio setup
//=========================================  3.Gyro setup

void setup() {
Serial.begin(115200);


}

void loop() {
  

}
//=========================================  4.Initialization (Homing)
//=========================================  5.Internal Data acquisition 
//=========================================  6.External Controller data acquisition
//=========================================  7.Rudimentary PID for each servo
//=========================================  8.Servo movement assignment
 
