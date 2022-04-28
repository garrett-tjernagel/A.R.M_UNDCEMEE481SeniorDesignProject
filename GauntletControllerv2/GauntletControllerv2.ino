//=========================================Introduction
/*Code written by Garrett Tjernagel for
  UND CEM EE480-481 SP22 Senior Design Capstone Project
  Augmented Robotic Manipulator (A.R.M)
  Partners: Branson Elliot and William Prody

*/

//=========================================Sources, Inspiration, and Links
/*
  https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7309037/
  https://forum.arduino.cc/t/simple-nrf24l01-2-4ghz-transceiver-demo/405123/2
  https://www.instructables.com/How-to-use-a-Flex-Sensor-Arduino-Tutorial/
  https://adafruit.github.io/Adafruit_MPU6050/html/class_adafruit___m_p_u6050.html#a64e6b74741d31138fb60f14ec2e7d9c1
  https://arduino.stackexchange.com/questions/86031/adafruit-mpu-6050-and-adafruit-i2c-multiplexer
  https://forum.arduino.cc/t/integration-of-acceleration/158296/10
  https://forum.arduino.cc/t/measuring-time/96602/4



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

//=========================================Gyroscope Declarations and Variables
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

int samplingTime = 1; //Enter time in ms

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

//=========================================Gyro Offsets
float x1RotOffset = 0.041478744;
float y1RotOffset = -0.048496174;
float z1RotOffset = -0.059629592;

float x2RotOffset = -0.924217463;
float y2RotOffset = -0.030702496;
float z2RotOffset = -0.01280241;

unsigned long deltaTime = 0;
unsigned long prevTime = 0;

//=========================================Flex Sensor Variables
const int thumbPin = A0;
const int pointerFingerPin = A1;
const int middleFingerPin = A2;
const int ringFingerPin = A3;
const int pinkyFingerPin = A4;

int pfFlex, tfFlex, mfFlex, rfFlex, pifFlex;

//=========================================Datapack/Radio Declarations + Variables
#define CE_PIN  7
#define CSN_PIN 8

const byte slaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

int shoulderPitch, shoulderYaw, elbowAngle, forearmRoll, wristPitch = 0;
int pack1 = 1;
int pack2 = 2;

int dataToSend1[6] = {shoulderPitch, shoulderYaw, elbowAngle, forearmRoll, wristPitch, pack1};
int dataToSend2[6] = {tfFlex, pfFlex, mfFlex, rfFlex, pifFlex, pack2};

//========================================= Configuration Variables
int potPin = A11;
int potUpperLim = 180;
int potLowerLim = 45;



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

//=========================================Setup
//=========================================
//=========================================
void setup(void) {
  //Reset Variables
  zeroSystem();
  Serial.begin(115200);

  //encoderSetup();

  Serial.begin(115200);
  //while (!Serial)
  delay(10); // will pause Zero, Leonardo, etc until serial console opens (DELETE THIS FOR THE FINAL ITERATIONS)

  setupGyros();
  //setupRadio();
  Serial.println("Starting Radio Tx");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setRetries(3, 5); // delay, count
  radio.openWritingPipe(slaveAddress);


}

void loop() {
  int i;
  //getGyroAccel();
  getGyroData();
  getFlexData();
  //encoderRun();
  potRead();

  
    Serial.print("Datapack GP:\t");
    Serial.print(dataToSend1[0]);
    Serial.print("\t");
    Serial.print(dataToSend1[1]);
    Serial.print("\t");
    Serial.print(dataToSend1[2]);
    Serial.print("\t");
    Serial.print(dataToSend1[3]);
    Serial.print("\t");
    Serial.print(dataToSend1[4]);
    Serial.print("\t >>>");
    Serial.print(sizeof(dataToSend1));
  
  Serial.print("Datapack2:\t");
  Serial.print(dataToSend2[0]);
  Serial.print("\t");
  Serial.print(dataToSend2[1]);
  Serial.print("\t");
  Serial.print(dataToSend2[2]);
  Serial.print("\t");
  Serial.print(dataToSend2[3]);
  Serial.print("\t");
  Serial.print(dataToSend2[4]);
  Serial.print("\t");
  Serial.print(dataToSend2[5]);
  Serial.print("\t >>>");
  Serial.println(sizeof(dataToSend2));



  sendData();
}
//=========================================
//=========================================
//=========================================

//=========================================Flex Sensor Data
void getFlexData() {
  tfFlex = analogRead(thumbPin);
  tfFlex = map(tfFlex, 0, 1023, 0, 100);

  pfFlex = analogRead(pointerFingerPin);
  pfFlex = map(pfFlex, 0, 1023, 0, 100);

  mfFlex = analogRead(middleFingerPin);
  mfFlex = map(mfFlex, 0, 1023, 0, 100);

  rfFlex = analogRead(ringFingerPin);
  rfFlex = map(rfFlex, 0, 1023, 0, 100);

  pifFlex = analogRead(pinkyFingerPin);
  pifFlex = map(pifFlex, 0, 1023, 0, 100);

  /*
    Serial.print("Finger Flex::: Thumb:\t");
    Serial.print(tfFlex);
    Serial.print("\t");
    Serial.print(pfFlex);
    Serial.print("\t");
    Serial.print(mfFlex);
    Serial.print("\t");
    Serial.print(rfFlex);
    Serial.print("\t");
    Serial.print(pifFlex);
    Serial.print("\t");
  */

  //dataToSend2[1] = tfFlex;
  //dataToSend2[0] = pfFlex;
  dataToSend2[1] = 0;
  dataToSend2[0] = 0;
  dataToSend2[2] = mfFlex;
  dataToSend2[4] = rfFlex;
  dataToSend2[3] = pifFlex;

  //Serial.println();
}

//=========================================Potentiameter
void potRead() {
  int potValue = analogRead(potPin);
  dataToSend1[2] = map(potValue, 0, 827, potLowerLim, potUpperLim);
  //Serial.print(potValue);
  //Serial.print("\t");
  //Serial.println(dataToSend1[2]);

}

//=========================================Gyro Data
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
    //Looping data points
    dataToSend1[0] = yrc1;
    dataToSend1[1] = xrc1;
    dataToSend1[3] = xrc2;
    dataToSend1[4] = yrc2;

    prevTime = newTime;
    void  reset(void);
    //Serial.println();
  }
}

//=========================================Data transmission

void sendData() {
  bool rslt1, rslt2;
  rslt1 = radio.write( &dataToSend1, sizeof(dataToSend1) );

  //Serial.print("Data Sent");
  if (rslt1) {
    Serial.print("A.R.M Rx1: Valid\t");
    //add led for good data
    //digitalWrite(greenLED, HIGH);
    //digitalWrite(redLED, LOW);
    
    rslt2 = radio.write( &dataToSend2, sizeof(dataToSend2) );
    if (rslt2) {
      Serial.print("A.R.M Rx 2: Valid\t");
      //add led for good data
      //digitalWrite(greenLED, HIGH);
      //digitalWrite(redLED, LOW);
    } else {
      //Serial.print("Tx failed\t");
      //add led for bad data here
      //digitalWrite(greenLED, LOW);
      //digitalWrite(redLED, HIGH);
    }

    Serial.println();
  }
}

//=========================================Gyro Setup
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
  //while (!Serial.available());

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
      delay(10);
  }
}

//=========================================Radio Setup
void setupRadio() {
  Serial.println("Starting Radio Tx");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setRetries(3, 5); // delay, count
  radio.openWritingPipe(slaveAddress);
}
