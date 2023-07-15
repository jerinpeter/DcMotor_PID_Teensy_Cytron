#include <Encoder.h>
#include <PID_v1.h>
#include "CytronMotorDriver.h"

// PID parameters
double input = 0.0;  // PID input (desired speed)
double output = 0.0;  // PID output (PWM value)
double setpoint = 0.0;  // PID setpoint (target speed)

// Encoder pins
const int encoderPinA = 2;
const int encoderPinB = 3;

// Encoder object
Encoder encoder(encoderPinA, encoderPinB);

// Motor object
CytronMD motor(PWM_DIR, 6, 7);


// PID parameters
double kp = 72.5;  // Proportional gain 
double ki = 0.001;  // Integral gain
double kd = 0.1;  // Derivative gain

// PID object
PID speedPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Variables
volatile long encoderCount = 0;  // Encoder count
double wheelCircumference = 0.2;  // Wheel circumference in meters
double encoderResolution = 1200.0;  // Encoder resolution


unsigned long previousMillis = 0;  // Previous loop time
unsigned long loopTime = 100;  // Loop time in milliseconds

void setup() {
  
  Serial.begin(115200);
  encoder.write(0);

  // Configure PID
  speedPID.SetMode(AUTOMATIC);  // Set PID to automatic mode
  speedPID.SetOutputLimits(-255, 255);  // Set PWM limits

  // Set initial setpoint and target speed
  setpoint = 0.85;  // Target speed in meters per second
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= loopTime) {
    // Read the encoder count
    long newEncoderCount = encoder.read();

    // Compute the time elapsed since the last iteration
    double timeElapsed = (currentMillis - previousMillis) / 1000.0;  // Convert to seconds

    // Compute the speed in meters per second
    double speed = (double)(newEncoderCount - encoderCount) * wheelCircumference / (encoderResolution * timeElapsed);
    encoderCount = newEncoderCount;

    // Set the PID input to the current speed
    input = speed;

    // Compute the PID output
    speedPID.Compute();

    // Apply the PID output as PWM value to the motor
        motor.setSpeed(output);
        Serial.print(output);
        Serial.println("");
 
    previousMillis = currentMillis;
  }
}
