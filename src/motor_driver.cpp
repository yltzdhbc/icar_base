#include "motor_driver.h"
// #include "config.h"

// #include <SoftwareSerial.h>

// SoftwareSerial softSerial1(6, 5); // rx tx
// #define DEBUG_SERIAL softSerial1

// #define MOTOR_DRIVER_DEBUG
#include <SoftwareSerial.h>
extern SoftwareSerial softSerial1; // rx tx
#define DEBUG_SERIAL softSerial1


volatile int32_t steps1;
volatile int32_t steps2;
int8_t dir_M1, dir_M2;
int16_t speed_M1, speed_M2;

StepperDriver::StepperDriver()
{
}

StepperDriver::~StepperDriver()
{
}

bool StepperDriver::init()
{
  pinMode(PIN_MOTOR_EN, OUTPUT); // ENABLE MOTORS
  pinMode(PIN_STEP_1, OUTPUT);   // STEP MOTOR 1
  pinMode(PIN_DIR_1, OUTPUT);    // DIR MOTOR 1
  pinMode(PIN_STEP_2, OUTPUT);   // STEP MOTOR 2
  pinMode(PIN_DIR_2, OUTPUT);    // DIR MOTOR 2
  motor_disable();

  // MOTOR1 => TIMER1
  TCCR1A = 0;                          // Timer1 CTC mode 4, OCxA,B outputs disconnected
  TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler=8, => 2Mhz
  OCR1A = ZERO_SPEED;                  // Motor stopped
  dir_M1 = 0;
  TCNT1 = 0;

  // MOTOR2 => TIMER3
  TCCR3A = 0;                          // Timer3 CTC mode 4, OCxA,B outputs disconnected
  TCCR3B = (1 << WGM32) | (1 << CS31); // Prescaler=8, => 2Mhz
  OCR3A = ZERO_SPEED;                  // Motor stopped
  dir_M2 = 0;
  TCNT3 = 0;
  delay(200);

  // Enable stepper drivers and TIMER interrupts
  motor_enable();

  // Enable TIMERs interrupts
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
  TIMSK3 |= (1 << OCIE1A); // Enable Timer3 interrupt

  return true;
}

float wheel_l, wheel_r;
float motor_l, motor_r;

bool StepperDriver::controlMotor(float wheel_radius, float wheel_separation, float velocity[2])
{

  wheel_l = velocity[LINEAR] - velocity[ANGULAR] * wheel_separation / 2; // m/s
  wheel_r = velocity[LINEAR] + velocity[ANGULAR] * wheel_separation / 2;

  motor_l = wheel_l * 60 / (2 * PI * wheel_radius); // r/min
  motor_r = wheel_r * 60 / (2 * PI * wheel_radius); // r/min

  setMotorSpeedM1(motor_l);
  setMotorSpeedM2(motor_r);
  return true;
}

//
bool StepperDriver::setMotorSpeedM1(int16_t rpm)
{
  long timer_period;
  int16_t speed;

  if ((speed_M1 - rpm) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - rpm) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = rpm;

  speed = speed_M1;

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    timer_period = RPM_TO_SPD / speed;
    dir_M1 = 1;
    SET_DIR_1;
  }
  else
  {
    timer_period = - RPM_TO_SPD / speed;
    dir_M1 = -1;
    CLR_DIR_1;
  }
  if (timer_period > 65535)
    timer_period = ZERO_SPEED;

#ifdef MOTOR_DRIVER_DEBUG
  DEBUG_SERIAL.print("[M1] ");
  DEBUG_SERIAL.print(" speed:");
  DEBUG_SERIAL.print(speed);
  DEBUG_SERIAL.print(" timer_period:");
  DEBUG_SERIAL.print(timer_period);
  DEBUG_SERIAL.print(" freq:");
  DEBUG_SERIAL.print(2000000 / timer_period);
  DEBUG_SERIAL.print(" rpm:");
  DEBUG_SERIAL.println(0.01875 * 2000000 / timer_period);
#endif

  OCR1A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT1 > OCR1A)
    TCNT1 = 0;

  return true;
}

bool StepperDriver::setMotorSpeedM2(int16_t rpm)
{
  long timer_period;
  int16_t speed;

  if ((speed_M2 - rpm) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - rpm) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = rpm;

  speed = speed_M2;

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = RPM_TO_SPD / speed;
    dir_M2 = 1;
    CLR_DIR_2;
  }
  else
  {
    timer_period = - RPM_TO_SPD / speed;
    dir_M2 = -1;
    SET_DIR_2;
  }
  if (timer_period > 65535)
    timer_period = ZERO_SPEED;

#ifdef MOTOR_DRIVER_DEBUG
  DEBUG_SERIAL.print("[M2] ");
  DEBUG_SERIAL.print(" speed:");
  DEBUG_SERIAL.print(speed);
  DEBUG_SERIAL.print(" timer_period:");
  DEBUG_SERIAL.print(timer_period);
  DEBUG_SERIAL.print(" freq:");
  DEBUG_SERIAL.print(2000000 / timer_period);
  DEBUG_SERIAL.print(" rpm:");
  DEBUG_SERIAL.println(0.01875 * 2000000 / timer_period);
#endif
  OCR3A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT3 > OCR3A)
    TCNT3 = 0;

  return true;
}

void StepperDriver::motor_enable(void)
{
  digitalWrite(PIN_MOTOR_EN, LOW);
}

void StepperDriver::motor_disable(void)
{
  digitalWrite(PIN_MOTOR_EN, HIGH);
}

// TIMER 1 : STEPPER MOTOR1 SPEED CONTROL
ISR(TIMER1_COMPA_vect)
{
  if (dir_M1 == 0)
    return;
  SET_STEP_1;
  if (dir_M1 > 0)
    steps1++;
  else
    steps1--;
  CLR_STEP_1;
}

// TIMER 3 : STEPPER MOTOR2 SPEED CONTROL
ISR(TIMER3_COMPA_vect)
{
  if (dir_M2 == 0)
    return;
  SET_STEP_2;
  if (dir_M2 > 0)
    steps2++;
  else
    steps2--;
  CLR_STEP_2;
}

// void StepperDriver::calcOdometry(Odom_t *Odom, uint32_t diff_time)
// {
//   float wheel_l, wheel_r; // 运动弧长 [rad]
//   float delta_s, delta_theta;
//   float step_time;

//   step_time = diff_time / 1000.0f; //MS转换为S
//   if (step_time == 0)
//     return;

//   wheel_l = (float)motor[0].step_diff * STEP_TO_RAD * WHEEL_RADIUS; //左轮运动弧长 m
//   wheel_r = (float)motor[1].step_diff * STEP_TO_RAD * WHEEL_RADIUS; //右轮运动弧长 m

//   delta_s = (wheel_l + wheel_r) / 2;                    //计算单位弧长
//   delta_theta = (wheel_r - wheel_l) / WHEEL_SEPARATION; //计算偏转角

//   Odom->Pos.theta += delta_theta;
//   Odom->Pos.y += delta_s * cos(Odom->Pos.theta);
//   Odom->Pos.x -= delta_s * sin(Odom->Pos.theta);

//   Odom->Vel.wz = (motor[1].vel - motor[0].vel) * RPM2MPS / WHEEL_SEPARATION; //单位 [m/s]
//   Odom->Vel.vx = (motor[1].vel + motor[0].vel) * RPM2MPS / 2;                //单位 [m/s]
//   Odom->Vel.vy = 0.0f;
// }
