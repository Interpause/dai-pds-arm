#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "env.h"

// Calibrated using: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/examples/oscillator/oscillator.ino
#define PWM_CAL_OSCI 29245000

#define SERVO_FREQ 50
#define SERVO_MIN_PULSE 100
#define SERVO_MAX_PULSE 2900
#define NUM_SERVOS 16
