#include <Wire.h>
#include <Arduino.h>

#include "./Sensor.cpp"

uint32_t lastUpdate = 0;
uint32_t now = 0;
float delta_t = 0.0f;

Sensor sensor;

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

  delta_t = ((now - lastUpdate) / 1000000.0f);

  SensorData *data = sensor.updateData(delta_t);

  Serial.print(data->roll);
  Serial.print('/');
  Serial.print(data->pitch);
  Serial.print('/');
  Serial.print(data->yaw);
  Serial.print('\n');

  lastUpdate = now;
}
