
/*
  4/9/22 - Gyroscopes are accurately reading and printing to serial seperately with the test functions. Next change is to impliment correct angle polarity, and packaging to the packet array.
  Will also start radio tranciever communication. After successful implimentation, will be transferring the test functions to the correct final functions.

*/

//=====================Libraries==========================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//=================Globals and Definitions=================
//Register Addressing
MPU6050 mpu(0x68);
MPU6050 mpu0(0x69);

//Interupt pin definition for the two Gyroscopes
#define INTERRUPT_PIN_1 2// use pin 2 for MPU
#define INTERRUPT_PIN_2 3// use pin 3 for MPU0
#define LED_PIN 13

#define OUTPUT_READABLE_YAWPITCHROLL

//Radio Transciever
#define CE_PIN   7
#define CSN_PIN 8

//Radio Address
const byte slaveAddress[5] = {'R','x','A','A','A'};

//Creating  a radio object
RF24 radio(CE_PIN, CSN_PIN);

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 1000; // send once per second

bool blinkState = false;

// Gryoscope Control/Status Variables
bool dmpReady = false;
bool dmp0Ready = false;// set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t mpu0IntStatus;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus0;
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize0;
uint8_t fifoBuffer[32]; // FIFO storage buffer
uint8_t fifoBuffer0[32];
uint16_t fifoCount;     // count of all bytes currently in FIFO

// Orientation and Motion Varaibles
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Quaternion q0;           // [w, x, y, z]         quaternion container
VectorInt16 aa0;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal0;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld0;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity0;    // [x, y, z]            gravity vector
float euler0[3];         // [psi, theta, phi]    Euler angle container
float ypr0[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
volatile bool mpu0Interrupt = false;     // indicates whether MPU0 interrupt pin has gone high

int Package[9];
int Home[9];
int CurrentPos[9];
int PrevPos[9];
int RecMesg;
bool VarButton = false;
int VarButtonPin = 2;




//====================Functions===================

void dmpDataReady() {
  mpuInterrupt = true;
  mpu0Interrupt = true;
}


void setup() {
  ComputerTestSteup();

  /*
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock.For Arduino UNO and other 400kHz board clocks
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true); //guessing that this would also tie into the 400kHz of UNOs
    #endif

    // Initialize Serial
    Serial.begin(115200);

    // Initialize Gyroscopes
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mpu0.initialize();
    pinMode(INTERRUPT_PIN_1, INPUT);
    pinMode(INTERRUPT_PIN_2, INPUT);
    // Initialize Gyroscopes
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mpu0.initialize();
    pinMode(INTERRUPT_PIN_1, INPUT);
    pinMode(INTERRUPT_PIN_2, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("Gyro connection successful") : F("Gyro connection failed"));
    Serial.println(mpu0.testConnection() ? F("Gyro-0 connection successful") : F("Gyro-0 connection failed"));

    GauntInint();
    ARMConEst();

    GauntHome();
  */


}
void ComputerTestSteup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);

  Serial.println("Radio Starting");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setRetries(3, 5); // delay, count
  radio.openWritingPipe(slaveAddress);

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  mpu0.initialize();
  pinMode(INTERRUPT_PIN_1, INPUT);
  pinMode(INTERRUPT_PIN_2, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(mpu0.testConnection() ? F("MPU6050-2 connection successful") : F("MPU6050-2 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  devStatus0 = mpu0.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);

  mpu0.setXGyroOffset(51);
  mpu0.setYGyroOffset(8);
  mpu0.setZGyroOffset(21);
  mpu0.setXAccelOffset(1150);
  mpu0.setYAccelOffset(-50);
  mpu0.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0 and devStatus0 == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    mpu0.CalibrateAccel(6);
    mpu0.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    mpu0.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpu0.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN_1));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN_2));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), dmpDataReady, RISING);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    mpu0IntStatus = mpu0.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    dmp0Ready = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    packetSize0 = mpu0.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.print(devStatus0);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void testMPU() {
  // if programming failed, don't try to do anything
  if (!dmpReady and !dmp0Ready) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer) and mpu0.dmpGetCurrentFIFOPacket(fifoBuffer0)) { // Get the Latest packet

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    mpu0.dmpGetQuaternion(&q0, fifoBuffer0);
    mpu0.dmpGetGravity(&gravity0, &q0);
    mpu0.dmpGetYawPitchRoll(ypr0, &q0, &gravity0);

    Serial.print("Shoulder\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print("\t");

    Serial.print("Wrist\t");
    Serial.print(ypr0[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr0[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr0[2] * 180 / M_PI);
    Serial.println();
#endif
  }
}


void loop() {
  getYPR();
  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    send();
    prevMillis = millis();
  }

  /*
    //Captures current positional Data from the Gauntlet
    DataPack();

    //send Package to ARM

    RecMesg = ErrorCheck();

    //wait for message for new packet
  */
}

void VarBut() {
  VarButton = false;
  while (VarButton = false) {
    if (digitalRead(VarButtonPin) == HIGH) {
      VarButtonPin = true;
    }
  }
}


void GauntHome() {
  int ShoulderX, ShoulderZ, ElbowZ, ForearmZ, Thumb, Pointer, Middle, Ring, Pinky;
  VarBut();
  //capture the current roll pitch and yaw data from the arm as the home position and use it in calculations
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu0.dmpGetQuaternion(&q0, fifoBuffer0);
  mpu.dmpGetEuler(euler, &q);
  mpu0.dmpGetEuler(euler, &q);
  mpu.dmpGetGravity(&gravity, &q);
  mpu0.dmpGetGravity(&gravity0, &q0);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu0.dmpGetYawPitchRoll(ypr0, &q0, &gravity0);

  //Set values to the correct containters and then set them all equal to their Home array position.
}

void getYPR() {
  // if programming failed, don't try to do anything
  if (!dmpReady and !dmp0Ready) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer) and mpu0.dmpGetCurrentFIFOPacket(fifoBuffer0)) { // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu0.dmpGetQuaternion(&q0, fifoBuffer0);
    mpu.dmpGetEuler(euler, &q);
    mpu0.dmpGetEuler(euler, &q);
    mpu.dmpGetGravity(&gravity, &q);
    mpu0.dmpGetGravity(&gravity0, &q0);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu0.dmpGetYawPitchRoll(ypr0, &q0, &gravity0);

    // Assign pertinet values to the pertnet variables.

  }
}

void send(){
  bool rslt;
    rslt = radio.write( &Package[0], sizeof(Package[0]) );
        // Always use sizeof() as it gives the size as the number of bytes.
        // For example if Package was an int sizeof() would correctly return 2

    Serial.print("Data Sent ");
    if (rslt) {
        Serial.println("  Acknowledge received");
    }
    else {
        Serial.println("  Tx failed");
    }
}

void ARMConEst() {
  //Connection routine between the ARM and Gauntlet
  Serial.print("Attempting Connection to A.R.M");
}

void DataPack() {
  //Collects data from sensors and creates a data pack.
  Serial.print("Data Pack");

}

int ErrorCheck() {
  //checks the message recieved for errors
  Serial.print("Error Message");
}

void GauntInint() {
  //Initialization for Gauntlet on startup or reset
  VarBut();

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  devStatus0 = mpu0.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  mpu0.setXGyroOffset(220);
  mpu0.setYGyroOffset(76);
  mpu0.setZGyroOffset(-85);
  mpu0.setZAccelOffset(1788);

  // make sure it worked (returns 0 if so)
  if ((devStatus == 0) and (devStatus0 == 0)) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu0.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu0.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu0.PrintActiveOffsets();

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpu0.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN_1));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN_2));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), dmpDataReady, RISING);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    mpu0IntStatus = mpu0.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    dmp0Ready = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    packetSize0 = mpu0.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.print(devStatus0);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

}
