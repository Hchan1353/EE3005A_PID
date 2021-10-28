/* Servo motor with Arduino example code. Position and sweep. More info: https://www.makerguides.com/ */
// Include the servo library:
#include <Servo.h>
// Create a new servo object:
Servo upperservo;
Servo lowerservo;
// Define the servo pin:
#define upperservoPin 9
#define lowerservoPin 10
// Create a variable to store the servo position:
int angle = 0;
void setup() {
  // Attach the Servo variable to a pin:
  upperservo.attach(upperservoPin);
  lowerservo.attach(lowerservoPin);
}
void loop() {
  // Tell the servo to go to a particular angle:
  /*upperservo.write(90);
  lowerservo.write(90);
  delay(1000);
  */
 /* upperservo.write(180);
  lowerservo.write(180);
  delay(1000);
  */
  upperservo.write(90);
  lowerservo.write(90);
  delay(1000);
  // Sweep from 0 to 180 degrees:
  /*for (angle = 0; angle <= 180; angle += 10) {
    upperservo.write(angle);
    lowerservo.write(angle);
    delay(15);
  }
  // And back from 180 to 0 degrees:
 /* for (angle = 180; angle >= 0; angle -= 10) {
    upperservo.write(angle);
    lowerservo.write(angle);
    delay(30);
  }*/
  //delay(1000);
}
