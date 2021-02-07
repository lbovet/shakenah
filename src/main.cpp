#include "M5Atom.h"
#include <OSCMessage.h>
#include <SLIPEncodedSerial.h>

long tprev;

#define MIN_SIZE 500.0
#define MAX_SIZE 30000.0
#define OSC

#ifdef OSC
SLIPEncodedSerial SLIPSerial(Serial);
#endif

double size = MIN_SIZE;
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
#ifdef OSC
  SLIPSerial.begin(115200);
#endif
  tprev = millis();
}

long step = 0;

void calc(Pea *pea, float acc, float dt)
{
  int hit = 0;
  double v = pea->v + acc * dt;
  double pos = pea->pos + pea->v * dt;
  if (pos < 0)
  {
    hit = pea->pos > 0 ? abs(v) : 0;
    pea->pos = 0;
    pea->v = 0;
  }
  else if (pos > size)
  {
    hit = pea->pos < size ? abs(v) : 0;
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
  return acc / 2 + h * angular / 2000;
}

void draw(int index, float pos, int hit)
{
  int p = (index * 5) + 5 * (pos / (size + 1));
  M5.dis.drawpix(p - 1, 0x000000);
  M5.dis.drawpix(p + 1, 0x000000);
  M5.dis.drawpix(p, hit > 0 ? (0x000033 + ((byte)min(255, 30 * (hit / 100) ^ 2))) * 256 : 0x000011);
}

void send(Pea peas[])
{
  OSCMessage msg("/shake");
  int sum = 0;
  for (int i = 0; i < 5; i++)
  {
    msg.add(peas[i].hit);
    sum += peas[i].hit;
  }
  if (sum > 0)
  {
#ifdef OSC
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
    size *= 2;
    if (size > MAX_SIZE)
    {
      size = MIN_SIZE;
    }
    for (int i = 0; i < size * 25 / (MAX_SIZE + 1); i++)
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

  step++;
  if (step % 20 == 0)
  {
    //Serial.printf("% 02.2f - % 02.2f,% 02.2f\n", gz, realAcc(0, ax, gz), realAcc(4, ax, gz));
  }

  tprev = t;
  M5.update();
}