#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
Servo servo;
Adafruit_MPU6050 mpu;

#define setPoint 0
#define Kp 2.5
#define Ki 0.5
#define Kd 1
#define motorPin 9

double previous_error = 0;
double integral = 0;
double output;

void setup() {
  servo.attach(motorPin);
Serial.begin(9600);
  servo.write(90); // Set the servo motor to the initial position
   while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  double angleX = a.acceleration.x; // Get the tilt angle of the platform along the X-axis
  double error = setPoint - angleX; // Calculate the error

  integral += error;
  double derivative = error - previous_error;

  output = Kp * error + Ki * integral + Kd * derivative;

  // Map the output to the servo motor position
  int servoPos = map(output, -90, 90, 0, 180);
  servo.write(servoPos);

  previous_error = error;

  delay(10);
}