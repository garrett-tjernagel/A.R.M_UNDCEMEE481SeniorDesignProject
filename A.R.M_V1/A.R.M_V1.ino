//=========================================Introduction
/*Code written by Garrett Tjernagel for
  UND CEM EE480-481 SP22 Senior Design Capstone Project
  Augmented Robotic Manipulator (A.R.M)
  Partners: Branson Elliott and William Prody

  4/23/22 1:05pm added initial radio comms for ARM

  Hello
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
Servo shoulderPitch1;
Servo shoulderPitch2;
Servo shoulderYaw;
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
int pitchPin1 = 13;
int pitchPin2 = 12;
int yawPin = 11;
int elbowPin = 10;
int forearmPin = 9;
int wristPin = 8;

int thumbPin = 0;
int pointerFingerPin = 0;
int middleFinerPin = 0;
int ringFingerPin = 0;
int pinkyFingerPin = 0;



//=========================================  2.Radio setup
#define CE_PIN   24
#define CSN_PIN 25

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};

RF24 radio(CE_PIN, CSN_PIN);

int dataReceived[5]; // this must match dataToSend in the TX
bool newData = false;

//=========================================  3.Gyro setup
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

//Gyro Number Containers
float gwxrs1, gwyrs1, gwzrs1 = 0;
float gwxrms1, gwyrms1, gwzrms1 = 0;
float xrc1, xrd1 = 0;
float yrc1, yrd1 = 0;
float zrc1, zrd1 = 0;

float gwxrs2, gwyrs2, gwzrs2 = 0;
float gwxrms2, gwyrms2, gwzrms2 = 0;
float xrc2, xrd2 = 0;
float yrc2, yrd2 = 0;
float zrc2, zrd2 = 0;

//Gyro Offsets
float x1RotOffset = 0.040768759;
float y1RotOffset = -0.048155658;
float z1RotOffset = -0.0491477;

float x2RotOffset = -0.951493925;
float y2RotOffset = 0.008503545;
float z2RotOffset = -0.021624156;

unsigned long deltaTime = 0;
unsigned long prevTime = 0;


void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens (DELETE THIS FOR THE FINAL ITERATIONS)

  setupGyros();


}

void loop() {
  //getGyroData();
    getData();
    showData();
}
//=========================================  4.Initialization (Homing)
//=========================================  5.Internal Data acquisition
//=========================================  6.External Controller data acquisition
//=========================================  7.Rudimentary PID for each servo
//=========================================  8.Servo movement assignment

void setupGyros() {
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu1.begin(0x68)) {
    Serial.println("Failed to find Gyro1");
    while (1) {
      delay(10);
    }
  }
  if (!mpu2.begin(0x69)) {
    Serial.println("Failed to find Gyro2");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!\n Enter anything to begin >");
  while (!Serial.available());

  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu1.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;

  }
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu1.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu2.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu1.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);
}

void getGyroData() {
  if (millis() < 1500) {
    zeroSystem();
  } else {
    /* Get new sensor events with the readings */
    sensors_event_t a1, g1, temp1;
    sensors_event_t a2, g2, temp2;
    mpu1.getEvent(&a1, &g1, &temp1);
    mpu2.getEvent(&a2, &g2, &temp2);

    //calculate average velocity between time samples
    long newTime = millis();
    deltaTime = (newTime - prevTime);
    /*
        Serial.print("\t");
        Serial.print("System Time (ms:)");
        Serial.print(deltaTime);
        Serial.print("\t");
    */

    gwxrs1 = g1.gyro.x + x1RotOffset;
    gwyrs1 = g1.gyro.y + y1RotOffset;
    gwzrs1 = g1.gyro.z + z1RotOffset;

    gwxrs2 = g2.gyro.x + x2RotOffset;
    gwyrs2 = g2.gyro.y + y2RotOffset;
    gwzrs2 = g2.gyro.z + z2RotOffset;

    /*
      Serial.print("Gyro W: X (rad/s): ");
      Serial.print(gwxrs1, 10);
      Serial.print("\t");
      Serial.print(gwyrs1, 10);
      Serial.print("\t");
      Serial.print(gwzrs1, 10);
      Serial.print("\t");

      Serial.print("Gyro2 W: X,Y,Z (rad/s): ");
      Serial.print("\t");
      Serial.print(gwxrs2, 10);
      Serial.print("\t");
      Serial.print(gwyrs2, 10);
      Serial.print("\t");
      Serial.print(gwzrs2, 10);
      Serial.print("\t");
    */

    gwxrms1 = gwxrs1 / 1000;
    gwyrms1 = gwyrs1 / 1000;
    gwzrms1 = gwzrs1 / 1000;

    gwxrms2 = gwxrs2 / 1000;
    gwyrms2 = gwyrs2 / 1000;
    gwzrms2 = gwzrs2 / 1000;
    /*
      Serial.print("Gyro1 W: X (rad/ms): ");
      Serial.print(gwxrms1, 10);
      Serial.print("\t");
      Serial.print(gwyrms1, 10);
      Serial.print("\t");
      Serial.print(gwzrms1, 10);
      Serial.print("\t");

      Serial.print("Gyro2 W: X (rad/ms): ");
      Serial.print(gwxrms2, 10);
      Serial.print("\t");
      Serial.print(gwyrms2, 10);
      Serial.print("\t");
      Serial.print(gwzrms2, 10);
      Serial.print("\t");
    */

    xrd1 = (gwxrms1 * deltaTime) * 180 / M_PI;
    yrd1 = (gwyrms1 * deltaTime) * 180 / M_PI;
    zrd1 = (gwzrms1 * deltaTime) * 180 / M_PI;

    xrd2 = (gwxrms2 * deltaTime) * 180 / M_PI;
    yrd2 = (gwyrms2 * deltaTime) * 180 / M_PI;
    zrd2 = (gwzrms2 * deltaTime) * 180 / M_PI;

    xrc1 = xrc1 + xrd1;
    yrc1 = yrc1 + yrd1;
    zrc1 = zrc1 + zrd1;

    xrc2 = xrc2 + xrd2;
    yrc2 = yrc2 + yrd2;
    zrc2 = zrc2 + zrd2;

    Serial.print("Gyro1 X,Y,Z (rad): ");
    Serial.print(xrc1, 4);
    Serial.print("\t");
    Serial.print(yrc1, 4);
    Serial.print("\t");
    Serial.print(zrc1, 4);
    Serial.print("\t");

    Serial.print("Gyro2 X,Y,Z (rad): ");
    Serial.print(xrc2, 4);
    Serial.print("\t");
    Serial.print(yrc2, 4);
    Serial.print("\t");
    Serial.print(zrc2, 4);
    Serial.print("\t");

    //Looping data points

    prevTime = newTime;
    void  reset(void);
    Serial.println();
  }
}

void getData() {
    if ( radio.available() ) {
        radio.read( &dataReceived, sizeof(dataReceived) );
        newData = true;
    }
}

void showData() {
    int i;
    if (newData == true) {
        Serial.print("Data received:\t");
        for(i=0;i<sizeof(dataReceived);i++){
        Serial.print(dataReceived[i]);
        Serial.print("/t");
        }
        Serial.println();
        newData = false;
    }
}

void zeroSystem() {
  //Gyro 1 (Shoulder)
  float gwxrs1, gwyrs1, gwzrs1 = 0;
  float gwxrms1, gwyrms1, gwzrms1 = 0;
  unsigned long deltaTime, to, tc = 0;
  float xrc1, xrd1 = 0;
  float yrc1, yrd1 = 0;
  float zrc1, zrd1 = 0;

  //Wrist
  float gwxrs2, gwyrs2, gwzrs2 = 0;
  float gwxrms2, gwyrms2, gwzrms2 = 0;
  float xrc2, xrd2 = 0;
  float yrc2, yrd2 = 0;
  float zrc2, zrd2 = 0;
}
