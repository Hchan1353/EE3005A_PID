#include<Arduino.h>
#include<Adafruit_MPU6050.h>
#include<PID_v1.h>
#include<Servo.h>

#define upperservoPin 9
#define lowerservoPin 10

int theta = 90;
int phi = 90;
int angleOffset=90;
//for testing use
int xSet[]={0,60,0,-60,0};
int ySet[]={0,0,30,0,-30};
int testCount=5;
int testStep=0;


Servo upperservo;
Servo lowerservo;
Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

void setup(void) {
  Serial.begin(115200);
  Serial.println("Hello");
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  // Try to initialize!
  /*
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  */
 mpu.begin();
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  mpu_temp = mpu.getTemperatureSensor();
  mpu_temp->printSensorDetails();

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();

  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();

  upperservo.attach(upperservoPin);
  lowerservo.attach(lowerservoPin);

  Serial.println("");
  delay(100);
}

void loop() {
if(testStep>=testCount){
  testStep=0;
}

upperservo.write(xSet[testStep]+angleOffset);
lowerservo.write(ySet[testStep]+angleOffset);
testStep++;





  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);


  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();
  delay(1000);
}
