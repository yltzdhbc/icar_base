
#ifndef _MOTOR_DRIVER_H_
#define _MOTOR_DRIVER_H_

#include "Arduino.h"




/* ********* 定义宏 ********* */

// #define DEBUG_SERIAL softSerial1


#define CLR(x, y) (x &= (~(1 << y)))
#define SET(x, y) (x |= (1 << y))

#define LINEAR 0
#define ANGULAR 1


/* ********* 具体配置区 ********* */
// 参数配置
#define STEP_ANGLE 1.8f // 步距角
#define SUBDIVIDE 16.0f // 细分数

#define MAX_ACCEL 10 // 电机的加速度 [RPM]

#define STEP_TO_RAD (STEP_ANGLE * PI) / 360 // 电机脉冲数与总弧度转换的系数

// 引脚配置
#define PIN_MOTOR_EN 8
#define PIN_STEP_1 2
#define PIN_DIR_1 5
#define PIN_STEP_2 3
#define PIN_DIR_2 6

// Arduino Cnc Shield
// #define SET_DIR_1 SET(PORTC, 6) // 5 --> PC6
// #define CLR_DIR_1 CLR(PORTC, 6)
// #define SET_STEP_1 SET(PORTD, 1) // 2 --> PD1
// #define CLR_STEP_1 CLR(PORTD, 1)
// #define SET_DIR_2 SET(PORTD, 7) // 6 --> PD7
// #define CLR_DIR_2 CLR(PORTD, 7)
// #define SET_STEP_2 SET(PORTD, 0) // 3 --> PD3
// #define CLR_STEP_2 CLR(PORTD, 0)

// mega
#define SET_DIR_1 SET(PORTE, 3) // 5 --> PE3
#define CLR_DIR_1 CLR(PORTE, 3)
#define SET_STEP_1 SET(PORTE, 4) // 2 --> PE4
#define CLR_STEP_1 CLR(PORTE, 4)
#define SET_DIR_2 SET(PORTH, 3) // 6 --> PH3
#define CLR_DIR_2 CLR(PORTH, 3)
#define SET_STEP_2 SET(PORTE, 5) // 3 --> E5
#define CLR_STEP_2 CLR(PORTE, 5)

/* ********* 计算宏 ********* */
#define RPM_TO_SPD (STEP_ANGLE * 60) / (SUBDIVIDE * 360) * 2000000 // rpm与定时器周期转换的系数

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489
#define ZERO_SPEED 65535
#define MOTOR_NUM 2

class StepperDriver
{
public:
  StepperDriver();  //构造函数
  ~StepperDriver(); //稀构函数
  bool init();
  bool controlMotor(float wheel_radius, float wheel_separation, float velocity[2]);
  bool setMotorSpeedM1(int16_t tspeed);
  bool setMotorSpeedM2(int16_t tspeed);
  void motor_enable(void);
  void motor_disable(void);

  // float wheel_rpm[MOTOR_NUM];
  // float wheel_velocity[MOTOR_NUM];

private:
  // uint32_t time_now, prev_time, d_time;

  // uint32_t baudrate_;
  // float protocol_version_;
  // uint8_t left_wheel_id_;
  // uint8_t right_wheel_id_;
  // bool torque_;
};

#endif

// #define PIN_MOTOR_EN 4
// #define PIN_STEP_1 7
// #define PIN_DIR_1 8
// #define PIN_STEP_2 12
// #define PIN_DIR_2 5
// SHIELD
// #define SET_DIR_1 SET(PORTB, 4)
// #define CLR_DIR_1 CLR(PORTB, 4)

// #define SET_DIR_2 SET(PORTC, 6)
// #define CLR_DIR_2 CLR(PORTC, 6)

// #define SET_STEP_1 SET(PORTE, 6)
// #define CLR_STEP_1 CLR(PORTE, 6)

// #define SET_STEP_2 SET(PORTD, 6)
// #define CLR_STEP_2 CLR(PORTD, 6)

// PD, // D0 - PD2
// PD,	// D1 - PD3
// PD, // D2 - PD1
// PD,	// D3 - PD0
// PD,	// D4 - PD4
// PC, // D5 - PC6
// PD, // D6 - PD7
// PE, // D7 - PE6

// PB, // D8 - PB4
// PB,	// D9 - PB5
// PB, // D10 - PB6
// PB,	// D11 - PB7
// PD, // D12 - PD6
// PC, // D13 - PC7

// PB,	// D14 - MISO - PB3
// PB,	// D15 - SCK - PB1
// PB,	// D16 - MOSI - PB2
// PB,	// D17 - SS - PB0

// PF,	// D18 - A0 - PF7
// PF, // D19 - A1 - PF6
// PF, // D20 - A2 - PF5
// PF, // D21 - A3 - PF4
// PF, // D22 - A4 - PF1
// PF, // D23 - A5 - PF0

// PD, // D24 / D4 - A6 - PD4
// PD, // D25 / D6 - A7 - PD7
// PB, // D26 / D8 - A8 - PB4
// PB, // D27 / D9 - A9 - PB5
// PB, // D28 / D10 - A10 - PB6
// PD, // D29 / D12 - A11 - PD6
// PD, // D30 / TX Led - PD5

// 7,	// A0				PF7					ADC7
// 6,	// A1				PF6					ADC6
// 5,	// A2				PF5					ADC5
// 4,	// A3				PF4					ADC4
// 1,	// A4				PF1					ADC1
// 0,	// A5				PF0					ADC0
// 8,	// A6		D4		PD4					ADC8
// 10,	// A7		D6		PD7					ADC10
// 11,	// A8		D8		PB4					ADC11
// 12,	// A9		D9		PB5					ADC12
// 13,	// A10		D10		PB6					ADC13
// 9	// A11		D12		PD6					ADC9
