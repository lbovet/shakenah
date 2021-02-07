#include "M5Atom.h"
#include <OSCMessage.h>
#include <SLIPEncodedSerial.h>
#include "network.h"
#include "WiFi.h"
#include <WiFiUdp.h>
#include "BluetoothSerial.h"
#include "SLIPEncodedStream.h"

WiFiUDP Udp;
IPAddress peer;
BluetoothSerial Bluetooth;

long tprev;

#define MIN_SIZE 1000.0
#define MAX_SIZE 5000.0
//#define SERIAL_OSC
//#define WIFI
#define BLUETOOTH

#ifdef SERIAL_OSC
SLIPEncodedSerial SLIPSerial(Serial);
#endif

#ifdef BLUETOOTH
SLIPEncodedStream SLIPStream(Bluetooth);
#endif

double size = (MAX_SIZE + MIN_SIZE) / 2;
struct Pea
{
  double pos;
  double v;
  int hit;
};

Pea peas[5];

void setup()
{
  M5.begin(true, false, true);
  delay(50);
  M5.dis.drawpix(0, 0xf00000);
  M5.IMU.Init();
#ifdef SERIAL_OSC
  SLIPSerial.begin(115200);
#endif
  tprev = millis();

#ifdef BLUETOOTH
  delay(500);
  Bluetooth.begin("Shakenah");
  Serial.println("Bluetooth Device is Ready to Pair");
#endif

#ifdef WIFI
  WiFi.begin(NETWORK_SSID, NETWORK_PWD);
  WiFi.setHostname("shakenah");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  WiFi.hostByName(NETWORK_PEER, peer);
#ifndef SERIAL_OSC
  Serial.print("Connected to the WiFi network. IP: ");
  Serial.println(WiFi.localIP());
#endif
#endif
}

long step = 0;

void calc(Pea *pea, float acc, float dt)
{
  int hit = 0;
  double v = pea->v + acc * dt;
  double pos = pea->pos + pea->v * dt;
  if (pos < 0)
  {
    hit = pea->pos > 0 ? v : 0;
    pea->pos = 0;
    pea->v = 0;
  }
  else if (pos > size)
  {
    hit = pea->pos < size ? v : 0;
    pea->pos = size;
    pea->v = 0;
  }
  else
  {
    pea->pos = pos;
    pea->v = v;
  }
  pea->hit = hit;
}

float realAcc(int h, float acc, float angular)
{
  return acc * 0.43; // + h*angular / 15000;
}

void draw(int index, float pos, int hit)
{
  int line = index * 5;
  int p = 5 * (pos / (size + 1));
  for (int i = 0; i < 5; i++)
  {
    if (i == p)
    {
      M5.dis.drawpix(line + p, hit > 0 ? (0x000033 + ((byte)min(255, 30 * (hit / 100) ^ 2))) * 256 : 0x220022);
    }
    else
    {
      M5.dis.drawpix(line + i, 0x000000);
    }
  }
}

void send(Pea peas[])
{
  OSCMessage msg("/shake");
  int sum = 0;
  for (int i = 0; i < 5; i++)
  {
    msg.add(peas[i].hit);
    sum += abs(peas[i].hit);
  }
  if (sum > 0)
  {
#ifdef BLUETOOTH
    SLIPStream.beginPacket();
    msg.send(SLIPStream);
    SLIPStream.endPacket();
#endif

#ifdef WIFI
    Udp.beginPacket(peer, 6101);
    msg.send(Udp);
    Udp.endPacket();
#endif
#ifdef SERIAL_OSC
    SLIPSerial.beginPacket();
    msg.send(SLIPSerial);
    SLIPSerial.endPacket();
#else
    Serial.printf("%i %i %i %i %i  \n",
                  msg.getInt(0), msg.getInt(1), msg.getInt(2), msg.getInt(3), msg.getInt(4));
#endif
    msg.empty();
  }
}

void loop()
{
  if (M5.Btn.wasPressed())
  {
    size += (MAX_SIZE - MIN_SIZE) / 5;
    if (size > MAX_SIZE)
    {
      size = MIN_SIZE;
    }
    for (int i = 0; i < (size - MIN_SIZE) * 5 / (MAX_SIZE - MIN_SIZE + 1); i++)
    {
      M5.dis.drawpix(i, 0xf00000);
    }
  }

  float gx, gy, gz, ax, ay, az;
  M5.IMU.getGyroData(&gx, &gy, &gz);
  M5.IMU.getAccelData(&ax, &ay, &az);

  long t = millis();
  long dt = t - tprev;

  for (int i = 0; i < 5; i++)
  {
    calc(&peas[i], realAcc(i, ax, gz), dt);
    if (!M5.Btn.isPressed())
    {
      draw(i, peas[i].pos, peas[i].hit);
    }
  }

  send(peas);

  tprev = t;
  M5.update();
}