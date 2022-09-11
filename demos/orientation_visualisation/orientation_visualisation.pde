import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

Serial myPort;
float roll, pitch, yaw;

float MAX_BOARD_ANGLE = 35;
float DEGREES_PER_TIMES_TO_BLINK = 3;

float prev_time = millis();

void setup() {
  size (640, 480, P3D);
  // IMPORTANT: specify your COM port
  myPort = new Serial(this, "/dev/cu.usbserial-1120", 9600);
  myPort.bufferUntil('\n');
}

float sign(float x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0; 
}

float square_fn(float freq, float t) {
  return sign(sin(2f * PI * t / (1f / freq)));
  //return 2f * (2f * floor(freq * t) - floor(2f * freq * t)) + 1f;
}

void draw() {
  translate(width/2, height/2, 0);

  float bend_value = abs(pitch);
  float constrained_bend = constrain(bend_value, 0, MAX_BOARD_ANGLE);
  float bend_overshoot = max(bend_value - MAX_BOARD_ANGLE, 0);

  float v = map(
    constrained_bend,
    0,
    MAX_BOARD_ANGLE,
    0,
    255
  );

  if (bend_overshoot > 0) {
    float blink_freq = round(bend_overshoot / DEGREES_PER_TIMES_TO_BLINK);
  
    if (square_fn(blink_freq, millis() / 1000f) > 0f) {
      v = 0;
    }
  }

  background(0, v, 0);
 
  // Rotate the object
  rotateY(radians(yaw));
  rotateX(radians(-pitch));
  rotateZ(radians(roll));

  fill(0, 76, 153);
 
  // Draw box
  box (386, 40, 200);
}

// Read data from the Serial Port
void serialEvent (Serial myPort) {
  String data = myPort.readStringUntil('\n');

  if (data != null) {
    String items[] = split(trim(data), ',');
   
    if (items.length > 1) {
      try {
        //--- Roll,Pitch in degrees
        roll = float(items[0]);
        pitch = float(items[1]);
        yaw = float(items[2]);
      } catch (RuntimeException e) {}
    }
  }
}
