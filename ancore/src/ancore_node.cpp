#include "ros/ros.h"
#include "ancore_node.h"

// #include "diagnostic_msgs/DiagnosticArray.h"
// #include "diagnostic_msgs/DiagnosticStatus.h"
// #include "diagnostic_msgs/KeyValue.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"

#include "std_srvs/Empty.h"

bool updateAlt(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void getAltCB(const ros::TimerEvent &);
void debugCB(const ros::TimerEvent &);
void cmdvelCB(const geometry_msgs::Twist &msg);

sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg;

geometry_msgs::Pose2D pose;
geometry_msgs::TransformStamped odom_trans;
geometry_msgs::Quaternion odom_quat;
std_msgs::Float64 altitude;

ros::Publisher imu_pub;
ros::Publisher odom_pub;
ros::Subscriber cmdvel_sub;
// ros::Publisher diagnostic_pub;
ros::Publisher alt_pub;

ros::Time last_time;
ros::Timer getAlt;

float wR = 0.0, wL = 0.0;

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
    std::string alt_topic;
    bool pubOdomFrame;

    nh_.param<std::string>("port", port, "/dev/ttySTM32");
    nh_.param<int>("baudrate", baudrate, 115200);
    nh_.param<float>("wheel_separation", wheel_separation, 0.5);
    nh_.param<float>("wheel_radius", wheel_radius, 0.075);
    nh_.param<float>("wheel_deadrad", wheel_deadrad, 0.7);

    nh_.param<float>("linear_min", linear_min, -0.5);
    nh_.param<float>("linear_max", linear_max, 0.5);
    nh_.param<float>("angular_min", angular_min, -0.5);
    nh_.param<float>("angular_max", angular_max, 0.5);
    nh_.param<bool>("publish_odom_frame", pubOdomFrame, false);

    nh_.param<std::string>("imu_frame_id", imu_msg.header.frame_id, "imu_link");
    nh_.param<std::string>("imu_topic", imu_topic, "imu");
    nh_.param<std::string>("altitude_topic", alt_topic, "altitude");
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
    alt_pub = nh.advertise<std_msgs::Float64>(alt_topic, 1);
    cmdvel_sub = nh.subscribe("cmd_vel", 10, &cmdvelCB);
    // diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

    ros::ServiceServer update_altitude = nh.advertiseService("update_altitude", updateAlt);
    getAlt = nh.createTimer(ros::Duration(0.5), getAltCB);
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
                                    messageBuffer = ser.read(48);
                                    memcpy(&protocal, &messageBuffer[0], 48);

                                    ros::Time current_time = ros::Time::now();

                                    float vR = float(protocal.SpeedR) / 10 * wheel_radius;
                                    float vL = float(protocal.SpeedL) / 10 * wheel_radius;

                                    // float vR = wR; //-1.0 * wheel_radius;
                                    // float vL = wL; //1.0 * wheel_radius;

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
                                    altitude.data = float(protocal.Altitude) / 1000;

                                    tf::Quaternion q(
                                        imu_msg.orientation.x,
                                        imu_msg.orientation.y,
                                        imu_msg.orientation.z,
                                        imu_msg.orientation.w);
                                    double dt = (current_time - last_time).toSec();
                                    pose.x += (trans_x * cos(pose.theta)) * dt;
                                    pose.y += (trans_x * sin(pose.theta)) * dt;
                                    pose.theta = tf::getYaw(q);
                                    // pose.theta += (rotat_z * dt);

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
                                // else
                                // {
                                //     ROS_WARN("Hi");
                                //     messageBuffer = ser.read(8);
                                //     memcpy(&Altitude, &messageBuffer[0], 4);
                                //     altitude.data = float(Altitude) / 1000;
                                //     alt_pub.publish(altitude);
                                // }
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

bool updateAlt(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    getAlt.start();
    uint8_t data[8];
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xEF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xEE;
    ser.write(&data[0], 8);

    return true;
}

void getAltCB(const ros::TimerEvent &)
{
    ROS_INFO("Publish altitude");
    alt_pub.publish(altitude);
    getAlt.stop();
}

void debugCB(const ros::TimerEvent &)
{
    // diagnostic_msgs::DiagnosticArray dia_array;
    // diagnostic_msgs::DiagnosticStatus ancore_status;
    // ancore_status.name = "Ancore";
    // ancore_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    // ancore_status.message = "Everything seem to be ok.";
    // diagnostic_msgs::KeyValue accuracy;
    // accuracy.key = "IMU accuracy";
    // accuracy.value = std::to_string(Accuracy);
    // diagnostic_msgs::KeyValue calibrate;
    // calibrate.key = "IMU Calibrate";
    // if (Calibrate == 1)
    // {
    //     calibrate.value = "true";
    // }
    // else
    // {
    //     calibrate.value = "false";
    // }

    // ancore_status.values.push_back(accuracy);
    // ancore_status.values.push_back(calibrate);

    // dia_array.status.push_back(ancore_status);
    // diagnostic_pub.publish(dia_array);
}

void cmdvelCB(const geometry_msgs::Twist &msg)
{
    float goal_trans_x = constrain(msg.linear.x, linear_min, linear_max);
    float goal_rotat_z = constrain(msg.angular.z, angular_min, angular_max);

    wL = (goal_trans_x - wheel_separation / 2.0 * goal_rotat_z) / wheel_radius;
    wR = -(goal_trans_x + wheel_separation / 2.0 * goal_rotat_z) / wheel_radius;
    // ROS_INFO_STREAM("Debug : " << wR << ", " << wL);
    // if (wL < wheel_deadrad && wL > 0)
    // {
    //     wL = wheel_deadrad;
    // }
    // else if (wL > -wheel_deadrad && wL < 0)
    // {
    //     wL = -wheel_deadrad;
    // }
    // if (wR < wheel_deadrad && wR > 0)
    // {
    //     wR = wheel_deadrad;
    // }
    // else if (wR > -wheel_deadrad && wR < 0)
    // {
    //     wR = -wheel_deadrad;
    // }

    uint8_t data[10];
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFE;
    data[4] = int(wR * 10) & 0xFF;
    data[5] = int(wL * 10) & 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFF;
    data[8] = 0xFF;
    data[9] = 0xEE;
    ser.write(&data[0], 10);
}