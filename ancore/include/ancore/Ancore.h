#include <iostream>
#include <sstream>
#include <cstring>

#include "serial/serial.h"

#define DEG_2_RAD 0.017453
#define G_2_MPSS 9.80665

using namespace std;

struct IAxis
{
	int32_t X;
	int32_t Y;
	int32_t Z;
};

struct IQuaternion
{
	int32_t W;
	int32_t X;
	int32_t Y;
	int32_t Z;
};

struct Protocal
{
	int8_t SpeedR;
	int8_t SpeedL;
	int16_t Accuracy;
	IAxis Accel;
	IAxis Gyro;
	IQuaternion Quat;
};

struct Information
{
	float speedR;
	float speedL;
	bool Accuracy;
};

class Ancore
{
  private:
	string messageBuffer;

  public:
	serial::Serial ser;
	Protocal pro;

	Ancore(string port, uint32_t baudrate);

	void readData();
	void send_raw_speed(float wR, float wL);

	// int16_t readInt16(char *bytes);
	// int32_t readInt32(char *byres);
};
