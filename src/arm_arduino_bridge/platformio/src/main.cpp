#include "main.h"

Adafruit_PWMServoDriver pwm;

void setAngle(uint8_t chan, int16_t ang)
{
  int16_t us = map(ang, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  us = constrain(us, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  Serial.print("Setting angle: ");
  Serial.println(us);
  pwm.writeMicroseconds(chan, us);
}

void connect()
{
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("Wifi connecting...");
  while (WiFi.status() != WL_CONNECTED)
  {
    neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);
    delay(500);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    Serial.print(".");
    delay(500);
  }
  Serial.println("");

  Serial.println("Wifi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(9600);
  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  connect();
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Wifi lost!");
    connect();
  }

  neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
  setAngle(0, -90);
  delay(5000);
  neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
  setAngle(0, 90);
  delay(5000);
  delay(10);
}
