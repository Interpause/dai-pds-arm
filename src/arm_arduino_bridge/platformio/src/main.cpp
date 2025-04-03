#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50
#define SERVO_MIN_PULSE 500
#define SERVO_MIN_ANGLE -90
#define SERVO_MAX_PULSE 2500
#define SERVO_MAX_ANGLE 90

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/** From -90 to 90 */
void setAngle(uint8_t chan, int16_t ang)
{
  int16_t us = map(ang, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  us = constrain(us, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  Serial.println(us);
  pwm.writeMicroseconds(chan, us);
}

void setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print("Waiting for serial...");
  while (!Serial)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    Serial.print(".");
  }
  Serial.println("");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);
}

void loop()
{
  Serial.println("Hello from Arduino!");
  neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
  setAngle(0, -90);
  delay(5000);
  neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);
  setAngle(0, 90);
  delay(5000);
}
