#include <Wire.h>
#include <Arduino.h>
#include <microLED.h>

#include "./Sensor.cpp"
#include "./ConstantBendColor.cpp"

uint32_t lastUpdate = 0;
uint32_t now = 0;
float delta_t = 0.0f;

Sensor sensor;

#define NUMLEDS 8
#define STRIP_PIN 2

microLED<NUMLEDS, STRIP_PIN, -1, LED_WS2812, ORDER_GBR> strip;

RGBColor from_color{255, 0, 0};
RGBColor to_color{0, 255, 0};

ConstantBendColor color_state(35, from_color, to_color);

void setup()
{
  Wire.begin();
  Serial.begin(9600);

  if (!sensor.setup())
  {
    Serial.print("Could not connect to MPU6050");

    // Loop forever if communication doesn't happen
    while (1)
    {
    }
  }
}

void loop()
{
  now = micros();
  delta_t = now - lastUpdate;
  lastUpdate = now;

  SensorData *data = sensor.updateData(delta_t);

  color_state.update(data->pitch, delta_t);

  RGBColor current_color = color_state.getCurrentColor();

  mData microled_color = mRGB(
      current_color.r,
      current_color.b,
      current_color.g);

  for (int i = 0; i < NUMLEDS; i++)
  {
    strip.set(i, microled_color);
  }

  strip.show();
}
