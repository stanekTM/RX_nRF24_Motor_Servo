
#ifndef __Config__
#define __Config__

#include <Arduino.h>

/*
  **************************************************************************************************************************************
  RC receiver configuration manual. See examples below "Custom configuration"
  **************************************************************************************************************************************
  Setting a unique address (5 bytes number or character). It is hardcoded into the firmware of RC transmitter openAVRc and Multiprotocol
  const byte address[6] = "jirka";
  
  RF communication channel setting. It is hardcoded into the firmware of RC transmitter openAVRc and Multiprotocol
  RF_CHANNEL  0 to 125 (2.4GHz to 2.525GHz)
  
  Setting the maximum nominal battery voltage
  BATTERY_VOLTAGE  4.2
  
  Setting the minimum battery voltage for alarm
  MONITORED_VOLTAGE  < 4.2
  
  Servo and motor output selection
  SERVO_12CH          Separate servo outputs (2 to 12 servo channels)
  MOTOR1, MOTOR2      Output for motor 1 and 2
  MIX_TANK_MOTOR1_2   Mixed "tank-arcade" output for motor 1 and 2
  SERVO_10CH_MOTOR1   Servo and motor 1 output (1 to 10 servo channels)
  SERVO_8CH_MOTOR1_2  Servo, motor 1 and motor 2 output (1 to 8 servo channels)
  
  Setting the number of servo channels
  SERVO_CHANNELS  1 to 12
  
  Setting the PWM prescaler according to the requirements and limitations of the timers/counters. Details in the "PWM" file
  PWM_30HZ --> PWM_62500HZ
  
  Setting the motor reaction point. Prevents initial rotor magnetic resistance
  REACTION_MOTOR1, REACTION_MOTOR2  0 to 255
  
  Setting the maximum motor power. Suitable for RC transmitters without endpoint setting
  MAX_FORWARD_MOTOR1, MAX_REVERSE_MOTOR1, MAX_FORWARD_MOTOR2, MAX_REVERSE_MOTOR2  0 to 255
  
  Brake setting, no brake 0, maximum brake 255
  BRAKE_MOTOR1, BRAKE_MOTOR2  0 to 255
  
  Pin settings specific to my PCB.
  PIN_LED
*/

//*********************************************************************************************************************
// Custom configuration for a specific RC model
//*********************************************************************************************************************
#define LET_L13_BLANIK
//#define BUGGY_1_32
//#define EACHINE_MONSTER
//#define FERARI_F40
//#define TANK_T34

// Glider Let L-13 Blanik
#if defined(LET_L13_BLANIK)
  
  const byte address[6] = "jirka";
  #define RF_CHANNEL  76
  
  #define BATTERY_VOLTAGE  4.2
  #define MONITORED_VOLTAGE  3.45
  
  #define SERVO_12CH

  #define SERVO_CHANNELS  4
#endif

// Buggy 1:32
#if defined(BUGGY_1_32)
  
  const byte address[6] = "jirka";
  #define RF_CHANNEL  76
  
  #define BATTERY_VOLTAGE  4.2
  #define MONITORED_VOLTAGE  3.45
  
  #define PIN_LED  2
  
  #define MOTOR1
  #define PWM_122HZ_TIMER2_3_11
  #define REACTION_MOTOR1  0
  #define MAX_FORWARD_MOTOR1  255
  #define MAX_REVERSE_MOTOR1  255
  #define BRAKE_MOTOR1  0
  
  #define MOTOR2
  #define PWM_122HZ_TIMER1_9_10
  #define REACTION_MOTOR2  0
  #define MAX_FORWARD_MOTOR2  255
  #define MAX_REVERSE_MOTOR2  255
  #define BRAKE_MOTOR2  0
#endif

// Eachine Monster
#if defined(EACHINE_MONSTER)
  
  const byte address[6] = "jirka";
  #define RF_CHANNEL  76
  
  #define BATTERY_VOLTAGE  4.2
  #define MONITORED_VOLTAGE  3.45
  
  #define MIX_TANK_MOTOR1_2
  
  #define PWM_122HZ_TIMER2_3_11
  #define REACTION_MOTOR1  0
  #define MAX_FORWARD_MOTOR1  255
  #define MAX_REVERSE_MOTOR1  255
  #define BRAKE_MOTOR1  255
  
  #define PWM_122HZ_TIMER1_9_10
  #define REACTION_MOTOR2  0
  #define MAX_FORWARD_MOTOR2  255
  #define MAX_REVERSE_MOTOR2  255
  #define BRAKE_MOTOR2  255
#endif

// Ferari F-40
#if defined(FERARI_F40)
  
  const byte address[6] = "jirka";
  #define RF_CHANNEL  76
  
  #define BATTERY_VOLTAGE  4.2
  #define MONITORED_VOLTAGE  3.45
  
  #define SERVO_10CH_MOTOR1
  
  #define SERVO_CHANNELS  1
  
  #define PWM_122HZ_TIMER2_3_11
  #define REACTION_MOTOR1  0
  #define MAX_FORWARD_MOTOR1  255
  #define MAX_REVERSE_MOTOR1  255
  #define BRAKE_MOTOR1  255
#endif

// Tank T-34/85
#if defined(TANK_T34)
  
  const byte address[6] = "jirka";
  #define RF_CHANNEL  76
  
  #define BATTERY_VOLTAGE  4.2
  #define MONITORED_VOLTAGE  3.45
  
  #define SERVO_8CH_MOTOR1_2
  
  #define SERVO_CHANNELS  1
  
  #define PWM_122HZ_TIMER2_3_11
  #define REACTION_MOTOR1  0
  #define MAX_FORWARD_MOTOR1  255
  #define MAX_REVERSE_MOTOR1  255
  #define BRAKE_MOTOR1  255
  
  #define PWM_488HZ_TIMER1_9_10_DEFAULT
  #define REACTION_MOTOR2  0
  #define MAX_FORWARD_MOTOR2  255
  #define MAX_REVERSE_MOTOR2  255
  #define BRAKE_MOTOR2  255
#endif

//*********************************************************************************************************************
// The number of RC channels of the receiver must match the RC channels of the transmitter
//*********************************************************************************************************************
#if defined(MOTOR1) || defined(MOTOR2) || defined(MIX_TANK_MOTOR1_2)
  #define MOTOR_CHANNELS  2 // Min/max. 2 channels
  #define SERVO_CHANNELS  0 // No servo channel. Need for definition
#endif

#if defined(SERVO_12CH)
  #define MOTOR_CHANNELS  0 // No motor channel. Need for definition
#endif

#if defined(SERVO_10CH_MOTOR1)
  #define MOTOR_CHANNELS  1 // Min. 1 channel
  #define MOTOR1
#endif

#if defined(SERVO_8CH_MOTOR1_2)
  #define MOTOR_CHANNELS  2 // Min. 2 channels
  #define MOTOR1
  #define MOTOR2
#endif

//*********************************************************************************************************************
// Radio data config (max. 32 bytes)
//*********************************************************************************************************************
// Received data array
const byte rc_channels = SERVO_CHANNELS + MOTOR_CHANNELS;

unsigned int rc_packet[rc_channels] = {1500};

// For one control channel with a value of 1000 to 2000 we need 2 bytes(packets)
const byte rc_packet_size = rc_channels * 2;

// Structure of sent ACK data
struct telemetry_packet_size
{
  byte rssi;
  byte batt_A1 = 255;
  byte batt_A2; // Not used yet
};
telemetry_packet_size telemetry_packet;

unsigned long rf_timeout = 0;

//*********************************************************************************************************************
// Dead zone adjustment of poor quality RC transmitter pots for motor control
//*********************************************************************************************************************
#define DEAD_ZONE  15

//*********************************************************************************************************************
// Setting the control range value
//*********************************************************************************************************************
#define MIN_CONTROL_VAL  1000
#define MID_CONTROL_VAL  1500
#define MAX_CONTROL_VAL  2000

#endif // End __Config__
 
