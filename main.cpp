#include<Arduino.h>
#include<Adafruit_MPU6050.h>
#include<PID_v1.h>
#include<Servo.h>
#include<remap.h>

#define upperservoPin 9
#define lowerservoPin 10

//Define Variables we'll be connecting to
double Setpoint_UP, Input_UP, Output_UP;
double Setpoint_LOW, Input_LOW, Output_LOW;
//Specify the links and initial tuning parameters
double Kp_UP=3, Ki_UP=0.00, Kd_UP=0.00;
double Kp_LOW=3, Ki_LOW=0.00, Kd_LOW=0.00;
PID upperPID(&Input_UP, &Output_UP, &Setpoint_UP, Kp_UP, Ki_UP, Kd_UP, DIRECT);
PID lowerPID(&Input_LOW, &Output_LOW, &Setpoint_LOW, Kp_LOW, Ki_LOW, Kd_LOW, DIRECT);
int angleOffset=90;


int xOffset,yOffset,zOffset;


Servo upperservo;
Servo lowerservo;
Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel, *mpu_gyro;
 sensors_event_t accel;
 sensors_event_t gyro;

void setup(void) {
  Serial.begin(115200);
  Serial.println("Hello");
  while (!Serial)
    delay(10); 
 mpu.begin();
  Serial.println("MPU6050 Found!");

  //mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  //mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  //mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);


  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();
  mpu_gyro = mpu.getGyroSensor();


Setpoint_UP = 128;
Setpoint_LOW = 128;

  //turn the PID on
  upperPID.SetMode(AUTOMATIC);
  lowerPID.SetMode(AUTOMATIC);

  upperservo.attach(upperservoPin);
  lowerservo.attach(lowerservoPin);

  upperservo.write(90);
  lowerservo.write(90);

  mpu_accel->getEvent(&accel);
int x=  accel.acceleration.x;
int y=  accel.acceleration.y;
int z=  accel.acceleration.z;

xOffset=x;
yOffset=y;
zOffset=z;
Serial.print("Offset:\tx:");
Serial.print(xOffset);
Serial.print("\ty:");
Serial.print(yOffset);
Serial.print("\tz:");
Serial.println(zOffset);


  Serial.println("Reseted");
  delay(1000);
}

void loop() {





 

  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);
int x=  accel.acceleration.x-xOffset;
int y=  accel.acceleration.y-yOffset;
int z=  accel.acceleration.z-zOffset;


  Serial.print("\t\tAccel X: ");
  Serial.print(x);
  Serial.print(" \tY: ");
  Serial.print(y);
  Serial.print(" \tZ: ");
  Serial.print(z);
  Serial.println(" m/s^2 ");

  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();



Input_UP=remapPIDIn(x);
Input_LOW=remapPIDIn(z);
upperPID.Compute();
lowerPID.Compute();
Serial.print("PID_UP:");
Serial.print(Output_UP);
Serial.print("\t");
Serial.println(remapPIDOut(Output_UP));
Serial.print("PID_LOW:");
Serial.print(Output_LOW);
Serial.print("\t");
Serial.println(remapPIDOut(Output_LOW));

upperservo.write(remapPIDOut(Output_UP));
lowerservo.write(remapPIDOut(Output_LOW));

  delay(100);
}
