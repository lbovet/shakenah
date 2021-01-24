#include "M5Atom.h"
#include <OSCMessage.h>
#include <SLIPEncodedSerial.h>

float accX = 0, accY = 0, accZ = 0;
float newX = 0, newY = 0, newZ = 0;

SLIPEncodedSerial SLIPSerial(Serial);

int light = 0;

void setup()
{
  M5.begin(true, false, true);
  delay(50);
  M5.dis.drawpix(0, 0xf00000);
  M5.IMU.Init();
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  SLIPSerial.begin(115200);
}

int window = 0;
int sum = 0;

void send(int shake)
{
  OSCMessage msg("/shake");
  msg.add(shake);

  SLIPSerial.beginPacket();
  msg.send(SLIPSerial);
  SLIPSerial.endPacket();
  msg.empty();
}

void detect(char axis, float previous, float current)
{
  float diff = current - previous;
  if (abs(diff) > 0.3)
  {
    int shake = (int)(abs(diff) * 100);
    sum += shake;
    light = min(light + shake, 2048);
    send(shake);
  }
}

void loop()
{
  M5.IMU.getAccelData(&newX, &newY, &newZ);

  detect('X', accX, newX);
  //detect('Y', accY, newY);
  //detect('Z', accZ, newZ);
  accX = newX;
  accY = newY;
  accZ = newZ;
  if (light > 16)
  {
    light -= 16;
  }
  for (int i = 0; i < 25; i++)
  {
    M5.dis.drawpix(i, min(light / 8, 255));
  }
  M5.update();
}