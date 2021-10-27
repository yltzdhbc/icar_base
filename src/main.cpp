// #define USE_USBCON

#include "config.h"

#include <std_msgs/String.h>
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";

#include <SoftwareSerial.h>
SoftwareSerial softSerial1(51, 53); // rx tx
#define DEBUG_SERIAL softSerial1

#define DEBUG

void setup()
{
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("debug start");

  nh.getHardware()->setBaud(115200);
  nh.initNode();

  nh.subscribe(cmd_vel_sub);
  // nh.subscribe(motor_power_sub);
  // nh.subscribe(reset_sub);

  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);

  tf_broadcaster.init(nh);

  // nh.advertise(chatter);

  initOdom();
  initJointStates();

  prev_update_time = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  motor_driver.init();
}

extern volatile int32_t steps1;
extern volatile int32_t steps2;
#define STEPPER_CTRL_TIMEOUT 5 //[s]

static uint32_t stp_timeout_cnt = 0;

void loop()
{
  uint32_t t = millis();

  updateTime();
  // updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

  if ((t - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();

    if (fabs(goal_velocity[LINEAR]) < 0.001 && fabs(goal_velocity[ANGULAR]) < 0.001)
    {
      stp_timeout_cnt++;
    }
    else
    {
      stp_timeout_cnt = 0;
      motor_driver.motor_enable();
    } 

    if (stp_timeout_cnt >= 100) // 20ms * 100 = 2000ms = 2s
    {
      motor_driver.motor_disable();
    }

    motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);

    tTime[0] = t;
  }

  if ((t - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    updateMotorInfo(steps1, steps2);
    publishDriveInformation();
    tTime[2] = t;
  }

#ifdef DEBUG
  if ((t - tTime[5]) >= (1000 / DEBUG_LOG_FREQUENCY))
  {
    sendDebuglog();
    tTime[5] = t;
  }
#endif

  // Send log message after ROS connection
  // sendLogMsg();

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}

/************************************ CallBack ******************************************/
void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR] = constrain(goal_velocity_from_cmd[LINEAR], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}

void motorPowerCallback(const std_msgs::Bool &power_msg)
{
  // bool dxl_power = power_msg.data;

  // motor_driver.setTorque(dxl_power);
}

void resetCallback(const std_msgs::Empty &reset_msg)
{
  char log_msg[50];

  (void)(reset_msg);

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  // sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);
}

/************************************ Publish ******************************************/
// void publishBatteryStateMsg(void)
// {
//   battery_state_msg.header.stamp = rosNow();
//   battery_state_msg.design_capacity = 1.8f; //Ah
//   // battery_state_msg.voltage = sensors.checkVoltage();
//   battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);

//   // if (battery_state == 0)
//   //   battery_state_msg.present = false;
//   // else
//   //   battery_state_msg.present = true;

//   battery_state_pub.publish(&battery_state_msg);
// }

void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/************************************ Time ******************************************/
ros::Time rosNow()
{
  return nh.now();
}

ros::Time addMicros(ros::Time &t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/************************************ Update ******************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};

  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index] = 0;
      last_rad[index] = 0.0;
      last_velocity[index] = 0.0;
    }

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT] = current_tick;
  last_rad[LEFT] += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT] = current_tick;
  last_rad[RIGHT] += TICK2RAD * (double)last_diff_tick[RIGHT];
}

void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT] = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT] = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

void updateGoalVelocity(void)
{
  goal_velocity[LINEAR] = goal_velocity_from_button[LINEAR] + goal_velocity_from_cmd[LINEAR] + goal_velocity_from_rc100[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_rc100[ANGULAR];

  // sensors.setLedPattern(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
}

void updateTF(geometry_msgs::TransformStamped &odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg);

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}

/************************************ Init ******************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index] = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

void initJointStates(void)
{
  // static char *joint_states_name[] = {(char *)"wheel_left_joint", (char *)"wheel_right_joint"};
  static char *joint_states_name[] = {(char *)"base_left_wheel_joint", (char *)"base_right_wheel_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name = joint_states_name;

  joint_states.name_length = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length = WHEEL_NUM;
}

/************************************ Calculate ******************************************/
bool calcOdometry(double diff_time)
{
  float *orientation;
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w; // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  // orientation = sensors.getOrientation();
  // theta = atan2f(orientation[1] * orientation[2] + orientation[0] * orientation[3],
  //                0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3]);

  // delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[2] += delta_theta;
  odom_pose[0] += delta_s * cos(odom_pose[2]);
  odom_pose[1] += delta_s * sin(odom_pose[2]);

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT] = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  // last_theta = theta;

  return true;
}

// /************************************ Calculate ******************************************/
// bool calcOdometry(double diff_time)
// {
//   float *orientation;
//   double wheel_l, wheel_r; // rotation value of wheel [rad]
//   double delta_s, theta, delta_theta;
//   static double last_theta = 0.0;
//   double v, w; // v = translational velocity [m/s], w = rotational velocity [rad/s]
//   double step_time;

//   wheel_l = wheel_r = 0.0;
//   delta_s = delta_theta = theta = 0.0;
//   v = w = 0.0;
//   step_time = 0.0;

//   step_time = diff_time;

//   if (step_time == 0)
//     return false;

//   wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
//   wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

//   if (isnan(wheel_l))
//     wheel_l = 0.0;

//   if (isnan(wheel_r))
//     wheel_r = 0.0;

//   delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
//   theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
//   // orientation = sensors.getOrientation();
//   // theta = atan2f(orientation[1] * orientation[2] + orientation[0] * orientation[3],
//   //                0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3]);

//   delta_theta = theta - last_theta;

//   // compute odometric pose
//   odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
//   odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
//   odom_pose[2] += delta_theta;

//   // compute odometric instantaneouse velocity

//   v = delta_s / step_time;
//   w = delta_theta / step_time;

//   odom_vel[0] = v;
//   odom_vel[1] = 0.0;
//   odom_vel[2] = w;

//   last_velocity[LEFT] = wheel_l / step_time;
//   last_velocity[RIGHT] = wheel_r / step_time;
//   last_theta = theta;

//   return true;
// }

/*******************************************************************************
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];

  String name = NAME;
  String firmware_version = FIRMWARE_VER;
  String bringup_log = "This core(v" + firmware_version + ") is compatible with TB3 " + name;

  const char *init_log_data = bringup_log.c_str();

  if (nh.connected())
  {
    if (log_flag == false)
    {
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
  }
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;

  if (isConnected)
  {
    if (wait_flag == false)
    {
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

extern float motor_l, motor_r;
extern float wheel_l, wheel_r;
/*******************************************************************************
* Send Debug data
*******************************************************************************/
void sendDebuglog(void)
{
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Steps ---- ");
  DEBUG_SERIAL.print("  STEPS1 : ");
  DEBUG_SERIAL.print(steps1);
  DEBUG_SERIAL.print("  STEPS2 : ");
  DEBUG_SERIAL.println(steps2);

  DEBUG_SERIAL.println("Odometry ---- ");
  DEBUG_SERIAL.print("  x : ");
  DEBUG_SERIAL.println(odom_pose[0]);
  DEBUG_SERIAL.print("  y : ");
  DEBUG_SERIAL.println(odom_pose[1]);
  DEBUG_SERIAL.print("  theta : ");
  DEBUG_SERIAL.println(odom_pose[2]);

  DEBUG_SERIAL.println("Cmd_vel ---- ");
  DEBUG_SERIAL.print("  liner : ");
  DEBUG_SERIAL.print(goal_velocity_from_cmd[LINEAR]);
  DEBUG_SERIAL.print("  angular : ");
  DEBUG_SERIAL.println(goal_velocity_from_cmd[ANGULAR]);

  DEBUG_SERIAL.print("  wheel_l [rpm] : ");
  DEBUG_SERIAL.print(wheel_l);
  DEBUG_SERIAL.print("  wheel_r [rpm] : ");
  DEBUG_SERIAL.println(wheel_r);

  DEBUG_SERIAL.print("  motor_l [rpm] : ");
  DEBUG_SERIAL.print(motor_l);
  DEBUG_SERIAL.print("  motor_l [rpm] : ");
  DEBUG_SERIAL.println(motor_r);

  // DEBUG_SERIAL.println("---------------------------------------");
  // DEBUG_SERIAL.println("EXTERNAL SENSORS");
  // DEBUG_SERIAL.println("---------------------------------------");
  // DEBUG_SERIAL.print("Bumper : "); DEBUG_SERIAL.println(sensors.checkPushBumper());
  // DEBUG_SERIAL.print("Cliff : "); DEBUG_SERIAL.println(sensors.getIRsensorData());
  // DEBUG_SERIAL.print("Sonar : "); DEBUG_SERIAL.println(sensors.getSonarData());
  // DEBUG_SERIAL.print("Illumination : "); DEBUG_SERIAL.println(sensors.getIlluminationData());

  // DEBUG_SERIAL.println("---------------------------------------");
  // DEBUG_SERIAL.println("OpenCR SENSORS");
  // DEBUG_SERIAL.println("---------------------------------------");
  // DEBUG_SERIAL.print("Battery : "); DEBUG_SERIAL.println(sensors.checkVoltage());
  // DEBUG_SERIAL.println("Button : " + String(sensors.checkPushButton()));

  // float* quat = sensors.getOrientation();

  // DEBUG_SERIAL.println("IMU : ");
  // DEBUG_SERIAL.print("    w : "); DEBUG_SERIAL.println(quat[0]);
  // DEBUG_SERIAL.print("    x : "); DEBUG_SERIAL.println(quat[1]);
  // DEBUG_SERIAL.print("    y : "); DEBUG_SERIAL.println(quat[2]);
  // DEBUG_SERIAL.print("    z : "); DEBUG_SERIAL.println(quat[3]);

  // DEBUG_SERIAL.println("---------------------------------------");
  // DEBUG_SERIAL.println("DYNAMIXELS");
  // DEBUG_SERIAL.println("---------------------------------------");
  // DEBUG_SERIAL.println("Torque : " + String(motor_driver.getTorque()));

  // int32_t encoder[WHEEL_NUM] = {0, 0};
  // motor_driver.readEncoder(encoder[LEFT], encoder[RIGHT]);

  // DEBUG_SERIAL.println("Encoder(left) : " + String(encoder[LEFT]));
  // DEBUG_SERIAL.println("Encoder(right) : " + String(encoder[RIGHT]));

  // DEBUG_SERIAL.println("---------------------------------------");
  // DEBUG_SERIAL.println("TurtleBot3");
  // DEBUG_SERIAL.println("---------------------------------------");
  // DEBUG_SERIAL.println("Odometry : ");
  // DEBUG_SERIAL.print("         x : "); DEBUG_SERIAL.println(odom_pose[0]);
  // DEBUG_SERIAL.print("         y : "); DEBUG_SERIAL.println(odom_pose[1]);
  // DEBUG_SERIAL.print("     theta : "); DEBUG_SERIAL.println(odom_pose[2]);
}
