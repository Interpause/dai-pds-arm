#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "env.h"

#define SERVO_FREQ 50
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define NUM_SERVOS 16

/** Connect to wifi. */
void connect();
