#include "MPU6050.h"

struct SensorData
{
  float yaw;
  float pitch;
  float roll;
};

/*
  Borrowed from: https://github.com/kriswiner/MPU6050/blob/5e55a39ee67cb729807d2fa0f4ae38994b9dceb1/MPU6050Library/MPU6050IMU.ino

  Hardware setup:
  MPU6050 Breakout --------- Arduino
  3.3V --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND
*/
class Sensor
{
private:
  MPU6050lib mpu;

  float SelfTest[6];

  // Bias corrections for gyro and accelerometer
  float gyroBias[3] = {0, 0, 0};
  float accelBias[3] = {0, 0, 0};

  // Stores the 16-bit signed accelerometer sensor
  int16_t accelCount[3];

  // scale resolutions per LSB for the sensors
  float aRes, gRes;

  // Stores the real accel value in g's
  float ax, ay, az;

  // Stores the real gyro value in degrees per seconds
  float gyrox, gyroy, gyroz;

  // Stores the 16-bit signed gyro sensor output
  int16_t gyroCount[3];

  // vector to hold quaternion
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  SensorData data;

  // parameters for 6 DoF sensor fusion calculations
  // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
  float GyroMeasError = PI * (40.0f / 180.0f);
  // compute beta
  float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
  // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
  float GyroMeasDrift = PI * (2.0f / 180.0f);
  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
  float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;
  // integration interval for both filter schemes
  float deltat = 0.0f;

  /**
    Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"

    (see http://www.x-io.co.uk/category/open-source/ for examples and more details)

    which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative

    device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.

    The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms

    but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
  */
  void MadgwickQuaternionUpdate(
      float delta_t,
      float ax,
      float ay,
      float az,
      float gyrox,
      float gyroy,
      float gyroz)
  {
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz; // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
      return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    // Compute and remove gyroscope biases
    gbiasx += gerrx * delta_t * zeta;
    gbiasy += gerry * delta_t * zeta;
    gbiasz += gerrz * delta_t * zeta;
    gyrox -= gbiasx;
    gyroy -= gbiasy;
    gyroz -= gbiasz;

    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
    qDot2 = _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
    qDot3 = _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
    qDot4 = _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 - (beta * hatDot1)) * delta_t;
    q2 += (qDot2 - (beta * hatDot2)) * delta_t;
    q3 += (qDot3 - (beta * hatDot3)) * delta_t;
    q4 += (qDot4 - (beta * hatDot4)) * delta_t;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
  }

public:
  uint8_t setup()
  {
    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);

    if (c == 0x68) // WHO_AM_I should always be 0x68
    {
      mpu.MPU6050SelfTest(SelfTest); // Start by performing self test

      if (
          SelfTest[0] < 1.0f &&
          SelfTest[1] < 1.0f &&
          SelfTest[2] < 1.0f &&
          SelfTest[3] < 1.0f &&
          SelfTest[4] < 1.0f &&
          SelfTest[5] < 1.0f)
      {
        mpu.calibrateMPU6050(gyroBias, accelBias);
        mpu.initMPU6050(); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

        // successfull initialization
        return 1;
      }
    }

    // couldn't connect to MPU6050
    return 0;
  }

  SensorData *updateData(float delta_t)
  {
    // If data ready bit set, all data registers
    // check if data ready interruptegisters have new data
    if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)
    {
      mpu.readAccelData(accelCount); // Read the x/y/z adc values
      aRes = mpu.getAres();

      // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0] * aRes; // get actual g value, this depends on scale being set
      ay = (float)accelCount[1] * aRes;
      az = (float)accelCount[2] * aRes;

      mpu.readGyroData(gyroCount); // Read the x/y/z adc values
      gRes = mpu.getGres();

      // Calculate the gyro value into actual degrees per second
      gyrox = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
      gyroy = (float)gyroCount[1] * gRes;
      gyroz = (float)gyroCount[2] * gRes;
    }

    MadgwickQuaternionUpdate(delta_t, ax, ay, az, gyrox * PI / 180.0f, gyroy * PI / 180.0f, gyroz * PI / 180.0f);

    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    data.yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    data.pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    data.roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    data.pitch *= 180.0f / PI;
    data.yaw *= 180.0f / PI;
    data.roll *= 180.0f / PI;

    return &data;
  }
};
