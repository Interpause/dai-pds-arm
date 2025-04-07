#include "main.h"

Adafruit_PWMServoDriver pwm;
AsyncWebServer server(80);
int16_t servoPulses[NUM_SERVOS];

void writeServos()
{
  for (auto i = 0; i < NUM_SERVOS; i++)
  {
    int16_t us = constrain(servoPulses[i], SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    Serial.printf("Servo %d: %d\n", i + 1, us);
    pwm.writeMicroseconds(i, us);
  }
}

void connect()
{
  Serial.printf("SSID: %s\n", WIFI_SSID);
  Serial.print("Wifi connecting...");
  while (WiFi.status() != WL_CONNECTED)
  {
    neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);
    delay(500);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    Serial.print(".");
    delay(500);
  }
  Serial.print("\n");

  Serial.println("Wifi connected!");
  Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Router address: %s\n", WiFi.gatewayIP().toString().c_str());
  Serial.printf("DNS address: %s\n", WiFi.dnsIP().toString().c_str());
  Serial.printf("Broadcast address: %s\n", WiFi.broadcastIP().toString().c_str());
  neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
}

void serverOnNotFound(AsyncWebServerRequest *req)
{
  if (req->method() == HTTP_OPTIONS)
    req->send(200);
  else
    req->send(404);
}

void serverOnRead(AsyncWebServerRequest *req)
{
  Serial.printf("Read request.");
  auto *res = req->beginResponseStream("application/json");

  res->printf("[%d", servoPulses[0]);
  for (auto i = 1; i < NUM_SERVOS; i++)
    res->printf(",%d", servoPulses[i]);
  res->print("]");
  req->send(res);
}

void serverOnWrite(AsyncWebServerRequest *req)
{
  Serial.println("Write request.");
  auto *res = req->beginResponseStream("text/plain");

  auto n_params = req->params();
  int16_t values[NUM_SERVOS];
  bool invalid = false;
  memcpy(values, servoPulses, sizeof(values));

  for (auto i = 0; i < n_params; i++)
  {
    auto param = req->getParam(i);
    auto name = param->name();
    auto value = param->value();
    auto id = name.toInt();
    auto pulsewidth = value.toInt();
    values[id - 1] = pulsewidth;

    if (!name.endsWith("_servo"))
    {
      res->printf("Invalid param name: %s (Should be NN_servo)\n", name.c_str());
      invalid = true;
    }
    if (id < 1 || id > NUM_SERVOS)
    {
      res->printf("Invalid servo id: %d (Should be 1 to %d)\n", id, NUM_SERVOS);
      invalid = true;
    }
    if (pulsewidth < SERVO_MIN_PULSE || pulsewidth > SERVO_MAX_PULSE)
    {
      res->printf("Invalid pulsewidth for %s: %d (Should be between %d and %d)\n", name.c_str(), pulsewidth, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
      invalid = true;
    }
  }

  if (invalid)
  {
    res->setCode(400);
  }
  else
  {
    res->print("OK");
    memcpy(servoPulses, values, sizeof(values));
    writeServos();
  }

  req->send(res);
}

void setupServer()
{
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.on("/write", HTTP_GET, serverOnWrite);
  server.on("/read", HTTP_GET, serverOnRead);
  server.onNotFound(serverOnNotFound);
  server.begin();
}

void setup()
{
  Serial.begin(9600);
  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);

  for (auto i = 0; i < NUM_SERVOS; i++)
    servoPulses[i] = SERVO_MIN_PULSE;

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  connect();

  setupServer();
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Wifi lost!");
    connect();
  }
  delay(10);
}
