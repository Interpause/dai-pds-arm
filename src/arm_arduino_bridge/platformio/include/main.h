#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include "env.h"

#define SERVO_FREQ 50
#define SERVO_MIN_PULSE 500
#define SERVO_MIN_ANGLE -90
#define SERVO_MAX_PULSE 2500
#define SERVO_MAX_ANGLE 90

/** From -90 to 90 degrees. */
void setAngle(uint8_t chan, int16_t ang);

/** Connect to wifi. */
void connect();
