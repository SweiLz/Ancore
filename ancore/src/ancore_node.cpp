#include "ros/ros.h"
#include "ancore_node.h"

#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

void debugCB(const ros::TimerEvent &);
void cmdvelCB(const geometry_msgs::Twist &msg);

sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg;

geometry_msgs::Pose2D pose;
geometry_msgs::TransformStamped odom_trans;
geometry_msgs::Quaternion odom_quat;

ros::Publisher imu_pub;
ros::Publisher odom_pub;
ros::Subscriber cmdvel_sub;
ros::Publisher diagnostic_pub;

ros::Time last_time;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ancore_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    std::string port;
    int baudrate;
    std::string imu_topic;
    std::string odom_topic;
    std::string cmd_topic;
    bool pubOdomFrame;

    nh_.param<std::string>("port", port, "/dev/ttySTM32");
    nh_.param<int>("baudrate", baudrate, 115200);
    nh_.param<float>("wheel_separation", wheel_separation, 0.5);
    nh_.param<float>("wheel_radius", wheel_radius, 0.075);
    nh_.param<bool>("publish_odom_frame", pubOdomFrame, false);

    nh_.param<std::string>("imu_frame_id", imu_msg.header.frame_id, "imu_link");
    nh_.param<std::string>("imu_topic", imu_topic, "imu");
    nh_.param<std::string>("odom_frame_id", odom_msg.header.frame_id, "odom");
    odom_trans.header.frame_id = odom_msg.header.frame_id;
    nh_.param<std::string>("odom_topic", odom_topic, "odom");
    nh_.param<std::string>("base_frame_id", odom_msg.child_frame_id, "base_footprint");
    odom_trans.child_frame_id = odom_msg.child_frame_id;
    nh_.param<std::string>("cmd_topic", cmd_topic, "cmd_vel");

    ser.setPort(port);
    ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
    ser.setTimeout(to);
    static tf::TransformBroadcaster odom_broadcaster;
    imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
    cmdvel_sub = nh.subscribe("cmd_vel", 10, &cmdvelCB);
    diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

    ros::Timer debug = nh.createTimer(ros::Duration(1.0), debugCB);

    last_time = ros::Time::now();
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
                        messageBuffer = "";
                        if (uint8_t(*ser.read(1).c_str()) == 0xFF)
                        {
                            if (uint16_t(*ser.read(2).c_str()) == 0xFFFF)
                            {
                                if (uint8_t(*ser.read(1).c_str()) == 0xFE)
                                {
                                    messageBuffer = ser.read(44);
                                    memcpy(&protocal, &messageBuffer[0], 44);
                                    ros::Time current_time = ros::Time::now();

                                    // float vR = float(protocal.SpeedR) / 10 * wheel_radius;
                                    // float vL = float(protocal.SpeedL) / 10 * wheel_radius;

                                    float vR = -1.0 * wheel_radius;
                                    float vL = 1.0 * wheel_radius;

                                    float trans_x = (-vR + vL) / 2.0;
                                    float rotat_z = (-vR - vL) / wheel_separation;
                                    Accuracy = protocal.Accuracy;
                                    if (Accuracy > 0)
                                    {
                                        Calibrate = 1;
                                    }

                                    imu_msg.header.stamp = current_time;
                                    imu_msg.linear_acceleration.x = float(protocal.Accel.X) / (1 << 16) * G_2_MPSS;
                                    imu_msg.linear_acceleration.y = float(protocal.Accel.Y) / (1 << 16) * G_2_MPSS;
                                    imu_msg.linear_acceleration.z = float(protocal.Accel.Z) / (1 << 16) * G_2_MPSS;

                                    imu_msg.angular_velocity.x = float(protocal.Gyro.X) / (1 << 16) * DEG_2_RAD;
                                    imu_msg.angular_velocity.y = float(protocal.Gyro.Y) / (1 << 16) * DEG_2_RAD;
                                    imu_msg.angular_velocity.z = float(protocal.Gyro.Z) / (1 << 16) * DEG_2_RAD;

                                    imu_msg.orientation.w = float(protocal.Quat.W) / (1 << 30);
                                    imu_msg.orientation.x = float(protocal.Quat.X) / (1 << 30);
                                    imu_msg.orientation.y = float(protocal.Quat.Y) / (1 << 30);
                                    imu_msg.orientation.z = float(protocal.Quat.Z) / (1 << 30);
                                    imu_pub.publish(imu_msg);

                                    tf::Quaternion q(
                                        imu_msg.orientation.x,
                                        imu_msg.orientation.y,
                                        imu_msg.orientation.z,
                                        imu_msg.orientation.w);
                                    double dt = (current_time - last_time).toSec();
                                    pose.x += (trans_x * cos(pose.theta)) * dt;
                                    pose.y += (trans_x * sin(pose.theta)) * dt;
                                    pose.theta = tf::getYaw(q);

                                    odom_quat = tf::createQuaternionMsgFromYaw(pose.theta);

                                    odom_trans.header.stamp = current_time;

                                    odom_trans.transform.translation.x = pose.x;
                                    odom_trans.transform.translation.y = pose.y;
                                    odom_trans.transform.rotation = odom_quat;

                                    if (pubOdomFrame)
                                    {
                                        odom_broadcaster.sendTransform(odom_trans);
                                    }

                                    odom_msg.header.stamp = current_time;
                                    odom_msg.pose.pose.position.x = pose.x;
                                    odom_msg.pose.pose.position.y = pose.y;
                                    odom_msg.pose.pose.orientation = odom_quat;

                                    odom_msg.twist.twist.linear.x = trans_x;
                                    odom_msg.twist.twist.angular.z = rotat_z;
                                    odom_pub.publish(odom_msg);
                                    // ROS_INFO_STREAM("Debug : " << pose.x << ", " << pose.y << ", " << pose.theta);

                                    last_time = current_time;
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

void debugCB(const ros::TimerEvent &)
{
    diagnostic_msgs::DiagnosticArray dia_array;
    diagnostic_msgs::DiagnosticStatus ancore_status;
    ancore_status.name = "Ancore";
    ancore_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    ancore_status.message = "Everything seem to be ok.";
    diagnostic_msgs::KeyValue accuracy;
    accuracy.key = "IMU accuracy";
    accuracy.value = std::to_string(Accuracy);
    diagnostic_msgs::KeyValue calibrate;
    calibrate.key = "IMU Calibrate";
    if (Calibrate == 1)
    {
        calibrate.value = "true";
    }
    else
    {
        calibrate.value = "false";
    }

    ancore_status.values.push_back(accuracy);
    ancore_status.values.push_back(calibrate);

    dia_array.status.push_back(ancore_status);
    diagnostic_pub.publish(dia_array);
}

void cmdvelCB(const geometry_msgs::Twist &msg)
{
    float goal_trans_x = msg.linear.x;
    float goal_rotat_z = msg.angular.z;

    float wL = (goal_trans_x - wheel_separation / 2.0 * goal_rotat_z) / wheel_radius;
    float wR = -(goal_trans_x + wheel_separation / 2.0 * goal_rotat_z) / wheel_radius;

    uint8_t data[4];
    data[0] = 0xFF;
    data[1] = int(wR * 10) & 0xFF;
    data[2] = int(wL * 10) & 0xFF;
    data[3] = 0xFE;
    ser.write(&data[0], 4);
}