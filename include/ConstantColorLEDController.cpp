#include <Arduino.h>
#include <microLED.h>

class ConstantColorLEDController
{
#define NUMLEDS 38
#define STRIP_PIN 2

private:
  microLED<NUMLEDS, STRIP_PIN, -1, LED_WS2812, ORDER_GBR> strip;

  unsigned long last_ms;

  void update_colors(mData color)
  {
    for (int i = 0; i < NUMLEDS; i++)
    {
      strip.set(i, color);
    }

    strip.show();
  }

public:
  void update(unsigned long current_ms, mData color)
  {
    if (current_ms - last_ms > 1000)
    {
      update_colors(color);
      last_ms = current_ms;
    }
  }
};