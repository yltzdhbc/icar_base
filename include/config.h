#ifndef _CORE_CONFIG_H_
#define _CORE_CONFIG_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
// #include <std_msgs/Int32.h>
// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
// #include <sensor_msgs/BatteryState.h>
// #include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "motor_driver.h"

#include <math.h>

#define RPM2MPS 0.00178023f     // 2PIR/60  R=0.017
#define MPS2RPM 561.723338f     // 60/2PIR  R=0.017

// #define WHEEL_RADIUS 0.047      // [m]
#define WHEEL_RADIUS 0.0469      // [m]
#define WHEEL_SEPARATION 0.135  // [m]

#define TURNING_RADIUS 0.1435   // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS 0.220      // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN -2147483648 // raw
#define ENCODER_MAX 2147483648  // raw

// #define MAX_LINEAR_VELOCITY (WHEEL_RADIUS * 2 * 3.14159265359 * 1000 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
// #define MAX_ANGULAR_VELOCITY (MAX_LINEAR_VELOCITY / TURNING_RADIUS)      // rad/s

#define MAX_LINEAR_VELOCITY     1000
#define MAX_ANGULAR_VELOCITY    1000

#define MIN_LINEAR_VELOCITY -MAX_LINEAR_VELOCITY
#define MIN_ANGULAR_VELOCITY -MAX_ANGULAR_VELOCITY

#define FIRMWARE_VER "1.2.3"
#define NAME "icar"

#define CONTROL_MOTOR_SPEED_FREQUENCY 50        //hz
#define CONTROL_MOTOR_TIMEOUT 500               //ms
#define IMU_PUBLISH_FREQUENCY 200               //hz
#define CMD_VEL_PUBLISH_FREQUENCY 50            //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY 100  //hz

#define VERSION_INFORMATION_PUBLISH_FREQUENCY 1 //hz
#define DEBUG_LOG_FREQUENCY 1                  //hz

#define WHEEL_NUM 2

#define LEFT 0
#define RIGHT 1

#define LINEAR 0
#define ANGULAR 1

#define DEG2RAD(x) (x * 0.01745329252) // *PI/180
#define RAD2DEG(x) (x * 57.2957795131) // *180/PI

// TICK= STEP   2   RAD
// 1 tick = 1/(200*16) * 360 du      = 0.1125[deg]
// = 0.1125[deg] * 3.14159265359 / 180 = 0.0019635f
#define TICK2RAD 0.0019635f

#define TEST_DISTANCE 0.300 // meter
#define TEST_RADIAN 3.14    // 180 degree



// #define DEBUG
// #define DEBUG_SERIAL SerialBT2

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg);
// void soundCallback(const turtlebot3_msgs::Sound &sound_msg);
void motorPowerCallback(const std_msgs::Bool &power_msg);
void resetCallback(const std_msgs::Empty &reset_msg);

// Function prototypes
// void publishCmdVelFromRC100Msg(void);
// void publishImuMsg(void);
// void publishMagMsg(void);
// void publishSensorStateMsg(void);
// void publishVersionInfoMsg(void);
void publishBatteryStateMsg(void);
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time &t, uint32_t _micros); // deprecated

// void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateTime(void);
void updateOdometry(void);
void updateJointStates(void);
void updateTF(geometry_msgs::TransformStamped &odom_tf);
// void updateGyroCali(bool isConnected);
void updateGoalVelocity(void);
void updateTFPrefix(bool isConnected);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);

void sendLogMsg(void);
void waitForSerialLink(bool isConnected);
void sendDebuglog(void);

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char *get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

// ros::Subscriber<turtlebot3_msgs::Sound> sound_sub("sound", soundCallback);

// ros::Subscriber<std_msgs::Bool> motor_power_sub("motor_power", motorPowerCallback);

ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
// turtlebot3_msgs::SensorState sensor_state_msg;
// ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// Version information of Turtlebot3
// turtlebot3_msgs::VersionInfo version_info_msg;
// ros::Publisher version_info_pub("firmware_version", &version_info_msg);

// IMU of Turtlebot3
// sensor_msgs::Imu imu_msg;
// ros::Publisher imu_pub("imu", &imu_msg);

// // Command velocity of Turtlebot3 using RC100 remote controller
// geometry_msgs::Twist cmd_vel_rc100_msg;
// ros::Publisher cmd_vel_rc100_pub("cmd_vel_rc100", &cmd_vel_rc100_msg);

// Odometry of Turtlebot3
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Battey state of Turtlebot3
// sensor_msgs::BatteryState battery_state_msg;
// ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// Magnetic field
// sensor_msgs::MagneticField mag_msg;
// ros::Publisher mag_pub("magnetic_field", &mag_msg);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
StepperDriver motor_driver;

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double last_rad[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double last_velocity[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
// Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
// Turtlebot3Controller controllers;
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_button[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
// Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
// bool setup_end = false;
// uint8_t battery_state = 0;

#endif // TURTLEBOT3_CORE_CONFIG_H_
