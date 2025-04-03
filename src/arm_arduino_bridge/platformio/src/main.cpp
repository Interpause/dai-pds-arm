#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  while (!Serial)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void loop()
{
  Serial.println("Hello from Arduino!");
  neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
  delay(1000);
  neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);
  delay(1000);
}
