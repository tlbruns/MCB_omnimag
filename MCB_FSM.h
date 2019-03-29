/*=========================================================================//

MCB_FSM.h

Implements a finite state machine for the Motor Control Board

Designed for use with Teensy 3.1/3.2 and Motor Control Board (Rev 1.2)


Trevor Bruns

Changelog-
04/10/2017: Initial Creation

//=========================================================================*/

#ifndef MCB_FSM_H
#define MCB_FSM_H

#include "MCB.h"
#include "MCBmodule.h"
#include <IntervalTimer.h>
#include <ros.h>
#include <Ethernet.h>
#include "WiznetHardware.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Vector3.h>
#include <medlab_motor_control_board/EnableMotor.h>
#include <medlab_motor_control_board/McbStatus.h>

#define MCB_VERSION 0.01

// Motor Control Board
//void motorSelectLedCallback(void); // ISR for toggling LED of selected motor during manual control state
void modeSwitchCallback(void); // ISR for mode switch on motherboard
void ampEnableISR(void); // ISR for MCP23008 interrupt that is triggered whenever an ampEnabled pin changes
void printErrorCode(MCB::ErrorCode errorCode); // reads MCB::getErrorCode() and prints message 
void blinkErrorCode(MCB::ErrorCode errorCode); // blinks green LEDs to signal error codes (1 Hz = WRONG_MODULE_ORDER; 2 Hz = ESTOP_TRIGGERED; 4 Hz = LIMIT_SWITCH_TRIGGERED_ON_STARTUP)
void timerRosCallback(void);

// Finite State Machine
enum MCBstate { statePowerUp, stateManualIdle, stateManualControl, stateRosInit, stateRosIdle, stateRosControl };
const char* MCBstateToString(MCBstate currentState);
MCBstate PowerUP(void);
MCBstate ManualIdle(void);
MCBstate ManualControl(void);
MCBstate RosInit(void);
MCBstate RosIdle(void);
MCBstate RosControl(void);
MCBstate stepStateMachine(MCBstate stateNext);

// PID Controller
//void timerPidCallback(void);

// ROS
//void timerRosCallback(void);
void subEnableRosControlCallback(const std_msgs::Bool& msg); // enters or exits RosControl state
void subEnableMotorCallback(const medlab_motor_control_board::EnableMotor& msg); // enables or disables a single motor
void subEnableAllMotorsCallback(const std_msgs::Bool& msg); // enables or disables all motors
void subEffortCommandCallback(const geometry_msgs::Vector3& msg); // callback for subscriber subEncoderCommand
void subResetDacsCallback(const std_msgs::Empty& msg); // re-initializes DACs
void subGetStatusCallback(const std_msgs::Empty& msg);

// Manual Control
//void timerManualControlCallback(void);
//void runManualControl(void);

#endif // !MCB_FSM_H