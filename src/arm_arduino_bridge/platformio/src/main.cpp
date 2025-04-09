#include "main.h"

Adafruit_PWMServoDriver pwm;
SimpleSerialProtocol ssp(Serial, SSP_BAUD, SSP_TIMEOUT, USB::onError, 'a', 'z');

uint16_t servoPulses[N_SERVO];
bool sent_err = false;
bool has_err = false;
unsigned long last_sent_servo = 0;
char err_msg[512];

void setupSSP()
{
  neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS); // Blue: connecting.
  ssp = SimpleSerialProtocol(Serial, SSP_BAUD, SSP_TIMEOUT, USB::onError, 'a', 'z');
  USB::setup(&ssp);
}

void handleConnUSB(bool is_connected)
{
  if (is_connected)
  {
    neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0); // Green: operational.
    // Send err msg if any, once.
    if (has_err && !sent_err)
    {
      USB::sendDebug(err_msg);
      sent_err = true;
    }
  }
  else
  {
    neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS); // Blue: reconnecting.
    sent_err = false;                                 // Allow err to be resent on reconnect.
  }
}

void writeServos()
{
  char msg[512];
  memset(msg, 0, sizeof(msg));
  appendf(msg, sizeof(msg), "[Write]: ");
  for (auto i = 0; i < N_SERVO; i++)
  {
    auto us = servoPulses[i];
    appendf(msg, sizeof(msg), " %d=%d", i + 1, us);
    pwm.writeMicroseconds(i, us);
  }
}

void setServos(uint16_t (&pulsewidths)[N_SERVO])
{
  for (auto i = 0; i < N_SERVO; i++)
  {
    auto us = pulsewidths[i];
    if (us < SERVO_MIN_PULSE)
      continue;
    us = constrain(pulsewidths[i], SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    servoPulses[i] = us;
  }
  writeServos();
}

void setup()
{
  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);

  // The below must be in exactly this order:
  // setPins, begin, setOscillatorFrequency, setPWMFreq
  Wire.setPins(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setOscillatorFrequency(PWM_CAL_OSCI);
  pwm.setPWMFreq(SERVO_FREQ);

  setupSSP();
  USB::h.callbackConnState = handleConnUSB;
  USB::h.setServos = setServos;
}

void loop()
{
  USB::loop();
  if (USB::has_fatal)
  {
    neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0); // Red: fatal error.
    setupSSP();
  }
  if (millis() - last_sent_servo > 1000 / SEND_SERVO_RATE)
  {
    last_sent_servo = millis();
    USB::sendServos(servoPulses);
  }
  delay(10);
}
