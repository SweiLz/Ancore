#include "ros/ros.h"
#include "serial/serial.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"

#define DEG_2_RAD 0.017453
#define G_2_MPSS 9.80665

serial::Serial ser;
std_msgs::Float32 current_msg;
sensor_msgs::Imu imu_msg;
std_msgs::Bool acc_msg;

int16_t reverse_2bytes(char *bytes);
int32_t reverse_4bytes(char *bytes);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ancore_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    std::string port;
    int baudrate = 115200;

    std::string imu_topic;
    std::string current_topic;

    nh_.param<std::string>("port", port, "/dev/ttyACM0");
    nh_.param<int>("baudrate", baudrate, 115200);
    nh_.param<std::string>("imu_frame_id", imu_msg.header.frame_id, "imu_link");
    nh_.param<std::string>("imu_topic", imu_topic, "imu");
    nh_.param<std::string>("current_topic", current_topic, "current");

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
    ros::Publisher current_pub = nh.advertise<std_msgs::Float32>(current_topic, 1000);
    ros::Publisher acc_pub = nh.advertise<std_msgs::Bool>("acc", 1);

    ser.setPort(port);
    ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
    ser.setTimeout(to);

    int count = 0;
    while (ros::ok())
    {
        try
        {
            ser.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port ");
        }

        if (ser.isOpen())
        {
            ROS_INFO_STREAM("Successfully connected to serial port.");
            try
            {
                while (ros::ok())
                {
                    if (ser.available())
                    {
                        std::string messageBuffer = "";
                        if (uint8_t(*ser.read(1).c_str()) == 0xFF)
                        {
                            if (uint16_t(*ser.read(2).c_str()) == 0xFFFF)
                            {
                                if (uint8_t(*ser.read(1).c_str()) == 0xFE)
                                {
                                    messageBuffer = ser.read(48);

                                    current_msg.data = double(reverse_2bytes(&messageBuffer[0]) / 1000.0);
                                    current_pub.publish(current_msg);

                                    acc_msg.data = (reverse_2bytes(&messageBuffer[2]) == 3) ? true : false;
                                    acc_pub.publish(acc_msg);

                                    imu_msg.header.stamp = ros::Time::now();
                                    imu_msg.linear_acceleration.x = (double)reverse_4bytes(&messageBuffer[4]) / (1 << 16) * G_2_MPSS;
                                    imu_msg.linear_acceleration.y = (double)reverse_4bytes(&messageBuffer[8]) / (1 << 16) * G_2_MPSS;
                                    imu_msg.linear_acceleration.z = (double)reverse_4bytes(&messageBuffer[12]) / (1 << 16) * G_2_MPSS;

                                    imu_msg.angular_velocity.x = (double)reverse_4bytes(&messageBuffer[16]) / (1 << 16) * DEG_2_RAD;
                                    imu_msg.angular_velocity.y = (double)reverse_4bytes(&messageBuffer[20]) / (1 << 16) * DEG_2_RAD;
                                    imu_msg.angular_velocity.z = (double)reverse_4bytes(&messageBuffer[24]) / (1 << 16) * DEG_2_RAD;

                                    imu_msg.orientation.w = (double)reverse_4bytes(&messageBuffer[28]) / (1 << 30);
                                    imu_msg.orientation.x = (double)reverse_4bytes(&messageBuffer[32]) / (1 << 30);
                                    imu_msg.orientation.y = (double)reverse_4bytes(&messageBuffer[36]) / (1 << 30);
                                    imu_msg.orientation.z = (double)reverse_4bytes(&messageBuffer[40]) / (1 << 30);
                                    imu_pub.publish(imu_msg);
                                }
                            }
                        }
                    }
                    ros::spinOnce();
                }
            }
            catch (const std::exception &e)
            {
                if (ser.isOpen())
                {
                    ser.close();
                }
                ROS_ERROR_STREAM(e.what());
                ROS_INFO_STREAM("Attempting reconnection after error.");
                ros::Duration(1.0).sleep();
            }
        }
        else
        {
            ROS_WARN_STREAM("Could not connect to serial device "
                            << port << ". Trying again every 1 second.");
            ros::Duration(1.0).sleep();
        }
    }

    return 0;
}

int16_t reverse_2bytes(char *bytes)
{
    int16_t out = 0;
    for (uint8_t i = 0; i < 2; i++)
    {
        int16_t byte = bytes[i] & 0xff;
        out |= byte << (8 * i);
    }
    return out;
}

int32_t reverse_4bytes(char *bytes)
{
    int32_t out = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        int32_t byte = bytes[i] & 0xff;
        out |= byte << (8 * i);
    }
    return out;
}