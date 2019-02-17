#include "Ancore.h"

Ancore::Ancore(string port, uint32_t baudrate)
{
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
    ser.setTimeout(to);
}

void Ancore::send_raw_speed(float wR, float wL)
{
    uint8_t data[4];
    data[0] = 0xFF;
    data[1] = int(wR * 10) & 0xFF;
    data[2] = int(wL * 10) & 0xFF;
    data[3] = 0xFE;
    ser.write(&data[0], 4);
}

void Ancore::readData()
{
    if (ser.available())
    {
        messageBuffer = "";

        if (uint8_t(*ser.read(1).c_str()) == 0xFF)
        {
            if (uint16_t(*ser.read(2).c_str()) == 0xFFFF)
            {
                if (uint8_t(*ser.read(1).c_str()) == 0xFE)
                {
                    messageBuffer = ser.read(44);
                    memcpy(&pro, &messageBuffer[0], 44);
                }
            }
        }
    }
}
// int16_t Ancore::readInt16(char *bytes)
// {
//     int16_t result;
//     memcpy(&result, bytes, sizeof(int16_t));
//     return result;
// }

// int32_t Ancore::readInt32(char *bytes)
// {
//     int32_t result;
//     memcpy(&result, bytes, sizeof(int32_t));
//     return result;
// }

// for (int i = 0; i < 44; i++)
// {
//     printf("%.2X ", uint8_t(messageBuffer[i]));
// }
// printf("\n");
// for (int i = 0; i < 44; i++)
// {
//     printf("%.2X ", uint8_t((pro + i)));
// }
// printf("\n");
// printf("%x\n", &messageBuffer[0]);

// pro.SpeedR = uint8_t(messageBuffer[0]) & 0xFF;
// for (int i = 0; i < 44; i++)
// {
//     printf("%.2X ", uint8_t(pro[i]));
// }
// printf("%x\n", pro.Accel);
// printf("\n\n");

// float speedR = float(messageBuffer[0]) / 10;
// printf("%f__\n", speedR);

//                                 speedL = float(messageBuffer[1]) / 10;

//                                 acc_msg.data = (reverse_2bytes(&messageBuffer[2]) == 3) ? true : false;
//                                 acc_pub.publish(acc_msg);

//                                 imu_msg.header.stamp = ros::Time::now();
//                                 imu_msg.linear_acceleration.x = (double)reverse_4bytes(&messageBuffer[4]) / (1 << 16) * G_2_MPSS;
//                                 imu_msg.linear_acceleration.y = (double)reverse_4bytes(&messageBuffer[8]) / (1 << 16) * G_2_MPSS;
//                                 imu_msg.linear_acceleration.z = (double)reverse_4bytes(&messageBuffer[12]) / (1 << 16) * G_2_MPSS;

//                                 imu_msg.angular_velocity.x = (double)reverse_4bytes(&messageBuffer[16]) / (1 << 16) * DEG_2_RAD;
//                                 imu_msg.angular_velocity.y = (double)reverse_4bytes(&messageBuffer[20]) / (1 << 16) * DEG_2_RAD;
//                                 imu_msg.angular_velocity.z = (double)reverse_4bytes(&messageBuffer[24]) / (1 << 16) * DEG_2_RAD;

//                                 imu_msg.orientation.w = (double)reverse_4bytes(&messageBuffer[28]) / (1 << 30);
//                                 imu_msg.orientation.x = (double)reverse_4bytes(&messageBuffer[32]) / (1 << 30);
//                                 imu_msg.orientation.y = (double)reverse_4bytes(&messageBuffer[36]) / (1 << 30);
//                                 imu_msg.orientation.z = (double)reverse_4bytes(&messageBuffer[40]) / (1 << 30);
//                                 imu_pub.publish(imu_msg);
//                             }