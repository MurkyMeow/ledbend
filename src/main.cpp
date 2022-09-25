#include <Wire.h>
#include <Arduino.h>

#include "Sensor.cpp"
#include "ConstantBendColor.cpp"
#include "ConstantColorLEDController.cpp"

uint32_t lastUpdate = 0;
uint32_t now = 0;
float delta_t = 0.0f;

Sensor sensor;

RGBColor from_color{0, 255, 255};
RGBColor to_color{255, 0, 0};

ConstantBendColor color_state(25, from_color, to_color);

ConstantColorLEDController led_controller;

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

  led_controller.update(
      now,
      mRGB(current_color.r, current_color.b, current_color.g));
}
