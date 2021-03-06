
#include <sstream>
#include "serial/serial.h"

#define constrain(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

#define DEG_2_RAD 0.017453
#define G_2_MPSS 9.80665

template <typename T>
struct Axis
{
    T X;
    T Y;
    T Z;
};

template <typename T>
struct Quaternion
{
    T W;
    T X;
    T Y;
    T Z;
};

struct Protocal
{
    int8_t SpeedR;
    int8_t SpeedL;
    int16_t Accuracy;
    Axis<int32_t> Accel;
    Axis<int32_t> Gyro;
    Quaternion<int32_t> Quat;
    int32_t Altitude;
};

serial::Serial ser;
Protocal protocal;
std::string messageBuffer;

float wheel_separation;
float wheel_radius;
float wheel_deadrad;

int8_t Accuracy;
int8_t Calibrate = 0;

float linear_min, linear_max;
float angular_min, angular_max;
