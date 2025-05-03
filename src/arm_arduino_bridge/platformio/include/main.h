#pragma once

#include "env.h"
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "USBridge.h"

// Calibrated using: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/examples/oscillator/oscillator.ino
#define PWM_CAL_OSCI 29245000

#define SERVO_FREQ 50
#define SERVO_MIN_PULSE 100
#define SERVO_MAX_PULSE 2900
#define N_SERVO 16
#define SEND_SERVO_RATE 50 // Hz

#define SSP_BAUD 921600
#define SSP_TIMEOUT 100 // ms

// We can max the brightness later but for now my eyes hurt. (0 to 255)
#define RGB_BRIGHTNESS 255

#define SDA_PIN 4
#define SCL_PIN 5
