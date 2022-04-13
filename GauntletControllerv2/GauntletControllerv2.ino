// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

int samplingTime = 1; //Enter time in ms

float gwxo1, gwyo1, gwzo1 = 0;
float gwxn1, gwyn1, gwzn1 = 0;
unsigned long deltaTime, to, tc = 0;
float xrc1, xrd1 = 0;
float yrc1, yrd1 = 0;
float zrc1, zrd1 = 0;

float xRotOffset=0.04;
float yRotOffset=0.00;
float zRotOffset=0.00;



void setup(void) {
//Reset Variables
float gwxo1, gwyo1, gwzo1 = 0;
float gwxn1, gwyn1, gwzn1 = 0;
unsigned long deltaTime, to, tc = 0;
float xrc1, xrd1 = 0;
  
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu1.begin(0x68) and !mpu2.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
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

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a1, g1, temp1;
  mpu1.getEvent(&a1, &g1, &temp1);

  //calculate average velocity between time samples

  unsigned long Time = millis();
  delay(samplingTime);
  unsigned long newTime = millis();
  unsigned long deltaTime = newTime - Time;
  Serial.print("Sampling Time (ms:)");
  Serial.print(deltaTime);
  Serial.print("\t");

  gwxn1 = g1.gyro.x;
  gwyn1 = g1.gyro.y;
  gwzn1 = g1.gyro.z;
  //Serial.print("\t");
  //Serial.print(gwxo1,10);
  //Serial.print("\t");
  //Serial.print(gwxn1,10);
  //Serial.print("\t");
  float avgXW1 = ((gwxo1 + gwxn1) / 2)+xRotOffset;
  float avgYW1 = ((gwyo1 + gwyn1) / 2)+yRotOffset;
  float avgZW1 = ((gwzo1 + gwzn1) / 2)+zRotOffset;
  
  //Serial.print("Current X,Y,Z Vel (rad/s) ");
  Serial.print(avgXW1,10);
  Serial.print("\t");
  Serial.print(avgYW1,10);
  Serial.print("\t");
  Serial.print(avgZW1,10);
  Serial.print("\t");


 xrd1 = avgXW1 * deltaTime;
 yrd1 = avgYW1 * deltaTime;
 zrd1 = avgZW1 * deltaTime; 
  xrc1 = xrc1 + xrd1;
  yrc1 = yrc1 + yrd1;
  zrc1 = zrc1 + zrd1;
  //Serial.print("Current X,Y,Z Pos (rad)");
  //Serial.print(xrc1);
  //Serial.print("\t");
  //Serial.print(yrc1);
  //Serial.print("\t");
  //Serial.print(zrc1);
  //Serial.print("\t");



gwxo1 = gwxn1;
gwyo1 = gwyn1;
gwzo1 = gwzn1;
  Serial.println();
}

/* OLD CODE FROM THE SKETCH: REMOVE AT END
  Print out the values
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g1.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g1.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g1.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");

*/
