//=========================================Introduction
/*Code written by Garrett Tjernagel for
  UND CEM EE480-481 SP22 Senior Design Capstone Project
  Augmented Robotic Manipulator (A.R.M)
  Partners: Branson Elliott and William Prody

  4/23/22 1:05pm added initial radio comms for ARM
  3:53pm organized some code, added servo testing to get the initial servo limits found out.
  4/26 added testing code for the servos + Servo limits

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
Servo shoulderServoPitch1;
Servo shoulderServoPitch2;
Servo shoulderServoYaw;
Servo elbowServo;
Servo forearmServo;
Servo wristServo;

//Hand Servos
Servo thumbServo;
Servo pointerServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

//Servo Pins
int pitchPin1 = 13;
int pitchPin2 = 12;
int yawPin = 11;
int elbowPin = 10;
int forearmPin = 9;
int wristPin = 8;

int thumbPin = 4;
int pointerPin = 7;
int middlePin = 6;
int ringPin = 5;
int pinkyPin = 3;

//Servo Limits
int  wristPitchUpperLim = 160;
int  wristPitchLowerLim = 100;

int  forearmRollUpperLim = 180;
int  forearmRollLowerLim = 0;

int  elbowPitchUpperLim = 140;
int  elbowPitchLowerLim = 10;

int  shoulderYawUpperLim = 180;
int  shoulderYawLowerLim = 10;

int  shoulderPitchUpperLim = 60;
int  shoulderPitchLowerLim = 25;

int thumbUpperLim = 110;
int thumbLowerLim = 0;

int pointerUpperLim = 110;
int pointerLowerLim = 0;

int middleUpperLim = 110;
int middleLowerLim = 0;

int ringUpperLim = 0;
int ringLowerLim = 110;

int pinkyUpperLim = 0;
int pinkyLowerLim = 110;

//servo Control
// 0=Shoulder Pitch 1=Shoulder Yaw 2=Elbow Pitch 3=Forearm Roll 4=Wrist Pitch
int servoCurrent[5];
int servoTarget[5];


//=========================================  2.Radio setup
#define CE_PIN   24
#define CSN_PIN 25

const byte thisSlaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};

RF24 radio(CE_PIN, CSN_PIN);

void setupRadio() {
  Serial.println("SimpleRx Starting");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();
}



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
float x1RotOffset = 0.102321597;
float y1RotOffset = -0.0483304;
float z1RotOffset = 0.012257273;

float x2RotOffset = 0.001032082;
float y2RotOffset = 0.028035659;
float z2RotOffset = -0.007918356;

unsigned long deltaTime = 0;
unsigned long prevTime = 0;


int redLED = 46;
int greenLED = 40;
int blueLED = 23;

//===============================================
//===============================================
//===============================================
void setup() {
  Serial.begin(115200);
  //while (!Serial)
  delay(10); // will pause Zero, Leonardo, etc until serial console opens (DELETE THIS FOR THE FINAL ITERATIONS)

  pinMode(redLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  digitalWrite(blueLED, HIGH);

  setupGyros();

  Serial.println("SimpleRx Starting");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();
  
  //setupServos();
  delay(10);
  digitalWrite(blueLED, LOW);
}

void loop() {
  //getGyroData();
  getData();
  showData();
  //servoTest();
  //servoManual();
  /*
    Serial.print("Angle Data:::Shoulder Pitch: ");
    Serial.print(servoCurrent[0]);
    Serial.print("\tShoulder Yaw: ");
    Serial.print(servoCurrent[1]);
    Serial.print("\tElbow Pitch: ");
    Serial.print(servoCurrent[2]);
    Serial.print("\tForearm Roll: ");
    Serial.print(servoCurrent[3]);
    Serial.print("\tWrist Pitch: ");
    Serial.print(servoCurrent[4]);
  */
  //Serial.println();
}
//===============================================
//===============================================
//=============================================== Manual Testing

//=========================================  Setup Codes
void setupServos() {

  shoulderServoPitch1.write(25);
  shoulderServoPitch1.write(25);
  shoulderServoPitch1.attach(pitchPin1);
  shoulderServoPitch2.attach(pitchPin2);
  shoulderServoYaw.write(10);
  delay(1);
  shoulderServoYaw.attach(yawPin);
  elbowServo.write(0);
  delay(1);
  elbowServo.attach(elbowPin);
  forearmServo.write(10);
  delay(1);
  forearmServo.attach(forearmPin);

  wristServo.write(90);
  delay(1);
  wristServo.attach(wristPin);

  thumbServo.write(0);
  delay(1);
  thumbServo.attach(thumbPin);
  pointerServo.write(0);
  delay(1);
  pointerServo.attach(pointerPin);
  middleServo.write(25);
  delay(1);
  middleServo.attach(middlePin);
  ringServo.write(180);
  delay(1);
  ringServo.attach(ringPin);
  pinkyServo.write(180);
  delay(1);
  pinkyServo.attach(pinkyPin);

}



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

      mpu2.setFilterBandwidth(MPU6050_BAND_21_HZ);
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
}

//=========================================  4.Initialization (Homing)
void initializeServos() {

}

//=========================================  5.Internal Data acquisition
void getGyroData() {
  //Wait 1.5 seconds before data aquisition
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
      Serial.print("Gyro W: X (rad/s): \t");
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

    /*
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
    */

    servoCurrent[0] = (int)( -1 * xrc2);
    servoCurrent[1] = (int) (-1 * yrc2);
    servoCurrent[2] = (int) (180 - (-1 * yrc1));
    servoCurrent[3] = (int) (xrc1);
    servoCurrent[4] = (int) (-1 * yrc1);


    //Looping data points
    prevTime = newTime;
    void  reset(void);
    Serial.println();
  }
}

//=========================================  6.External Controller data acquisition
void getData() {
  if ( radio.available() ) {
    radio.read( &dataReceived, sizeof(dataReceived) );
    newData = true;
    //Serial.println("Good");
    //digitalWrite(greenLED, HIGH);
    //digitalWrite(redLED, LOW);
  } else {
    //Serial.println("Bad");
    //digitalWrite(greenLED, LOW);
    //digitalWrite(redLED, HIGH);
  }

}

void showData() {

  if (newData == true) {
    Serial.print("Datapack:\t");
    Serial.print(dataReceived[0]);
    Serial.print("\t");
    Serial.print(dataReceived[1]);
    Serial.print("\t");
    Serial.print(dataReceived[2]);
    Serial.print("\t");
    Serial.print(dataReceived[3]);
    Serial.print("\t");
    Serial.print(dataReceived[4]);
    Serial.print("\t >>>");
    Serial.println(sizeof(dataReceived));
    newData = false;
  }
}
//=========================================  7.Rudimentary PID Calculations for each servo

//=========================================  8.Servo movement assignment

//=========================================  9. System Controls
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

void servoManual() {
  int i;
  int d = 1;

  while (true) {
    char servoChoice[2];

    Serial.print("Select Servo and Direction \nEnter as to sequential characters.\n");
    Serial.print("a > Shoulder Pitch\nb > Shoulder Yaw\nc > Elbow Pitch\nd > Forearm Roll\ne > Thumb\nf > Pointer\n g > Middle\nh >Ring\n i >Pinky");
    Serial.print("\nu > Upper Limit\nl > Lower Limit\n ");
    while (!Serial.available());
    if (Serial.available() <= 2) {
      servoChoice[0] = Serial.read();
      delay(1);
      servoChoice[1] = Serial.read();

      if (servoChoice[0] == 'a') {
        if (servoChoice[1] == 'u') {
          Serial.print("\nMoving Shoulder Pitch to upper limit.\n");
          for (i = shoulderPitchLowerLim; i < shoulderPitchUpperLim; i++) {
            shoulderServoPitch1.write(i);
            shoulderServoPitch2.write(i);
            Serial.println(i);
            delay(d);
          }

        } else if (servoChoice[1] == 'l') {
          Serial.print("\nMoving Shoulder Pitch to lower limit.\n");
          for (i = shoulderPitchUpperLim; i > shoulderPitchLowerLim; --i) {
            shoulderServoPitch1.write(i);
            shoulderServoPitch2.write(i);
            Serial.println(i);
            delay(d);
          }

        } else {
          Serial.println("Unknown Input\n");
        }
      } else if (servoChoice[0] == 'b') {
        if (servoChoice[1] == 'u') {
          Serial.print("\nMoving Shoulder Yaw to upper limit.\n");
          for (i = shoulderYawLowerLim; i < shoulderYawUpperLim; i++) {
            shoulderServoYaw.write(i);
            Serial.println(i);
            delay(d);
          }

        } else if (servoChoice[1] == 'l') {
          Serial.print("\nMoving Shoulder Yaw  to lower limit.\n");
          for (i = shoulderYawUpperLim; i > shoulderYawLowerLim; --i) {
            shoulderServoYaw.write(i);
            Serial.println(i);
            delay(d);
          }

        } else {
          Serial.println("Unknown Input\n");
        }
      } else if (servoChoice[0] == 'c') {
        if (servoChoice[1] == 'u') {
          Serial.print("\nMoving Elbow pitch to upper limit.\n");
          for (i = elbowPitchLowerLim; i < elbowPitchUpperLim; i++) {
            elbowServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else if (servoChoice[1] == 'l') {
          Serial.print("\nMoving Elbow pitch  lower limit.\n");
          for (i = elbowPitchUpperLim; i > elbowPitchLowerLim; --i) {
            elbowServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else {
          Serial.println("Unknown Input\n");
        }
      } else if (servoChoice[0] == 'd') {
        if (servoChoice[1] == 'u') {
          Serial.print("\nMoving Forearm Roll to upper limit.\n");
          for (i = forearmRollLowerLim; i < forearmRollUpperLim; i++) {
            forearmServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else if (servoChoice[1] == 'l') {
          Serial.print("\nMoving Forearm Roll to lower limit.\n");
          for (i = forearmRollUpperLim; i > forearmRollLowerLim; --i) {
            forearmServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else {
          Serial.println("Unknown Input\n");
        }
      } else if (servoChoice[0] == 'e') {
        if (servoChoice[1] == 'u') {
          Serial.print("\nMoving Thumb to upper limit.\n");
          for (i = thumbLowerLim; i < thumbUpperLim; i++) {
            thumbServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else if (servoChoice[1] == 'l') {
          Serial.print("\nMoving Thumbto lower limit.\n");
          for (i = thumbUpperLim; i > thumbLowerLim; --i) {
            thumbServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else {
          Serial.println("Unknown Input\n");
        }
      } else if (servoChoice[0] == 'f') {
        if (servoChoice[1] == 'u') {
          Serial.print("\nMoving pointer to upper limit.\n");
          for (i = pointerLowerLim; i < pointerUpperLim; i++) {
            pointerServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else if (servoChoice[1] == 'l') {
          Serial.print("\nMoving pointer to lower limit.\n");
          for (i = pointerUpperLim; i > pointerLowerLim; --i) {
            pointerServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else {
          Serial.println("Unknown Input\n");
        }
      } else if (servoChoice[0] == 'g') {
        if (servoChoice[1] == 'u') {
          Serial.print("\nMoving middle to upper limit.\n");
          for (i = middleLowerLim; i < middleUpperLim; i++) {
            middleServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else if (servoChoice[1] == 'l') {
          Serial.print("\nMoving middle to lower limit.\n");
          for (i = middleUpperLim; i > middleLowerLim; --i) {
            middleServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else {
          Serial.println("Unknown Input\n");
        }
      } else if (servoChoice[0] == 'h') {
        if (servoChoice[1] == 'u') {
          Serial.print("\nMoving ring to upper limit.\n");
          for (i = ringLowerLim; i > ringUpperLim; --i) {
            ringServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else if (servoChoice[1] == 'l') {
          Serial.print("\nMoving ring to lower limit.\n");
          for (i = ringUpperLim; i < ringLowerLim; i++) {
            ringServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else {
          Serial.println("Unknown Input\n");
        }
      } else if (servoChoice[0] == 'i') {
        if (servoChoice[1] == 'u') {
          Serial.print("\nMoving pinky to upper limit.\n");
          for (i = pinkyLowerLim; i > pinkyUpperLim; --i) {
            pinkyServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else if (servoChoice[1] == 'l') {
          Serial.print("\nMoving pinky to lower limit.\n");
          for (i = pinkyUpperLim; i < pinkyLowerLim; i++) {
            pinkyServo.write(i);
            Serial.println(i);
            delay(d);
          }

        } else {
          Serial.println("Unknown Input\n");
        }
      } else {
        Serial.println("\nUnknown Input");
      }
    } else {
      Serial.println("\nUnknown Input");
      serialFlush();
    }
  }


}
void serialFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}
