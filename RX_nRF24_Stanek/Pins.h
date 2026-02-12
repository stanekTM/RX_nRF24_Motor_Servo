
#ifndef __Pins__
#define __Pins__

#include <Arduino.h>
#include "Config.h"

//*****************************
// ATmega328P/PB pins overview
//*****************************
// PD0 - D0   PWM  328PB
// PD1 - D1   PWM  328PB
// PD2 - D2   PWM  328PB
// PD3 - D3   PWM
// PD4 - D4
// PD5 - D5   PWM
// PD6 - D6   PWM
// PD7 - D7
// PB0 - D8
// PB1 - D9   PWM
// PB2 - D10  PWM
// PB3 - D11  PWM  MOSI
// PB4 - D12       MISO
// PB5 - D13       SCK
// PC0 - D14 / A0
// PC1 - D15 / A1
// PC2 - D16 / A2
// PC3 - D17 / A3
// PC4 - D18 / A4   SDA
// PC5 - D19 / A5   SCL
// PB6 - D20        XTAL1
// PB7 - D21        XTAL2
// PC6 - D22        RESET
// PE0 - D23        328PB
// PE1 - D24        328PB
// PE2 - D25 / A6   328PB
// PE3 - D26 / A7   328PB
// ADC6   -    A6
// ADC7   -    A7

// PWM pins for motor 1 (possible combination, max. 2)
#if defined(MOTOR1)
  const byte pins_motor1[2] = {3, 11};
#endif

// PWM pins for motor 2 (possible combination, max. 2)
#if defined(MOTOR2)
  const byte pins_motor2[2] = {9, 10};
#endif

#if defined(MIX_TANK_MOTOR1_2)
  const byte pins_motor1[2] = {3, 11};
  const byte pins_motor2[2] = {9, 10};
#endif

// Pins for servos (possible combination, max. 12)
#if defined(SERVO_12CH)
  const byte pins_servo[12] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
#endif

// Pins for servos (possible combination, max. 10)
#if defined(SERVO_10CH_MOTOR1)
  const byte pins_servo[10] = {2, 4, 5, 6, 7, 8, 9, 10, 12, 13};
#endif

// Pins for servos (possible combination, max. 8)
#if defined(SERVO_8CH_MOTOR1_2)
  const byte pins_servo[8] = {2, 4, 5, 6, 7, 8, 12, 13};
#endif

// LED alarm
#if defined(PIN_LED)
  #else
  #define PIN_LED          A6
#endif

// Input battery
#define PIN_BATTERY        A7

// Pins for nRF24L01+
#define PIN_CE             A0
#define PIN_CSN            A1

// Software SPI https://nrf24.github.io/RF24/md_docs_2arduino.html
//          SCK            A2
//          MOSI           A3
//          MISO           A4

#endif
 
