#include <Arduino.h>

struct RGBColor
{
  float r;
  float g;
  float b;
};

class ConstantBendColor
{
private:
  float max_board_pitch;

  RGBColor current_color;

  RGBColor from_color;
  RGBColor to_color;

  float lerp(float from, float to, float by)
  {
    return from * (1 - by) + to * by;
  }

  float progress(float start, float end, float percentage)
  {
    return start + (end - start) * percentage;
  }

public:
  ConstantBendColor(
      float max_board_pitch,
      RGBColor from_color,
      RGBColor to_color)
  {
    this->max_board_pitch = max_board_pitch;
    this->from_color = from_color;
    this->to_color = to_color;
    this->current_color = from_color;
  }

  void update(float pitch, float delta_t)
  {
    float bend_value = abs(pitch);
    float constrained_bend = constrain(bend_value, 0, max_board_pitch);
    float bend_overshoot = max(bend_value - max_board_pitch, 0);

    float bend_percentage = constrained_bend / max_board_pitch;

    Serial.println(bend_percentage);

    current_color.r = progress(to_color.r, from_color.r, bend_percentage);
    current_color.b = progress(to_color.b, from_color.b, bend_percentage);
    current_color.g = progress(to_color.g, from_color.g, bend_percentage);
  }

  void setFromColor(RGBColor from_color)
  {
    this->from_color = from_color;
  }

  void setToColor(RGBColor to_color)
  {
    this->to_color = to_color;
  }

  RGBColor getCurrentColor()
  {
    return current_color;
  }
};
