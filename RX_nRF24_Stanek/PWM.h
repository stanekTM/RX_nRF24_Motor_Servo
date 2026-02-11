
//*********************************************************************************************************************
// PWM frequency prescaler for ATmega328P and ATmega328PB
//*********************************************************************************************************************

#ifndef __PWM_h__
#define __PWM_h__

#include <Arduino.h>
#include "Config.h"
#include "Pins.h"

//*********************************************************************************************************************
// Pin D5 and D6 (8-bit Timer/Counter 0, functions delay, millis, micros and delayMicroseconds)
//*********************************************************************************************************************
//#define PWM_61HZ_TIMER0_5_6
//#define PWM_244HZ_TIMER0_5_6
//#define PWM_976HZ_TIMER0_5_6_DEFAULT
//#define PWM_7812HZ_TIMER0_5_6
//#define PWM_62500HZ_TIMER0_5_6

#if defined(PIN_5_6_MOTOR1)
  #if defined(PWM_61HZ_TIMER0_5_6)
    #define PRESCALER_TIMER0_5_6  1024
  #endif
  #if defined(PWM_244HZ_TIMER0_5_6)
    #define PRESCALER_TIMER0_5_6  256
  #endif
  #if defined(PWM_976HZ_TIMER0_5_6_DEFAULT)
    #define PRESCALER_TIMER0_5_6  64
  #endif
  #if defined(PWM_7812HZ_TIMER0_5_6)
    #define PRESCALER_TIMER0_5_6  8
  #endif
  #if defined(PWM_62500HZ_TIMER0_5_6)
    #define PRESCALER_TIMER0_5_6  1
  #endif
#endif

//*********************************************************************************************************************
// Pin D9 and D10 (16-bit Timer/Counter 1, Servo library)
//*********************************************************************************************************************
//#define PWM_30HZ_TIMER1_9_10
//#define PWM_122HZ_TIMER1_9_10
//#define PWM_488HZ_TIMER1_9_10_DEFAULT
//#define PWM_3906HZ_TIMER1_9_10
//#define PWM_31250HZ_TIMER1_9_10

#if defined(PIN_9_10_MOTOR2)
  #if defined(PWM_30HZ_TIMER1_9_10)
    #define PRESCALER_TIMER1_9_10  1024
  #endif
  #if defined(PWM_122HZ_TIMER1_9_10)
    #define PRESCALER_TIMER1_9_10  256
  #endif
  #if defined(PWM_488HZ_TIMER1_9_10_DEFAULT)
    #define PRESCALER_TIMER1_9_10  64
  #endif
  #if defined(PWM_3906HZ_TIMER1_9_10)
    #define PRESCALER_TIMER1_9_10  8
  #endif
  #if defined(PWM_31250HZ_TIMER1_9_10)
    #define PRESCALER_TIMER1_9_10  1
  #endif
#endif

//*********************************************************************************************************************
// Pin D3 and D11 (8-bit Timer/Counter 2)
//*********************************************************************************************************************
//#define PWM_30HZ_TIMER2_3_11     1024
//#define PWM_122HZ_TIMER2_3_11
//#define PWM_244HZ_TIMER2_3_11    128
//#define PWM_488HZ_TIMER2_3_11_DEFAULT  64
//#define PWM_976HZ_TIMER2_3_11    32
//#define PWM_3906HZ_TIMER2_3_11   8
//#define PWM_31250HZ_TIMER2_3_11  1

#if defined(PIN_3_11_MOTOR3)
  #if defined(PWM_30HZ_TIMER2_3_11)
    #define PRESCALER_TIMER2_3_11  1024
  #endif
  #if defined(PWM_122HZ_TIMER2_3_11)
    #define PRESCALER_TIMER2_3_11  256
  #endif
  #if defined(PWM_244HZ_TIMER2_3_11)
    #define PRESCALER_TIMER2_3_11  128
  #endif
  #if defined(PWM_488HZ_TIMER2_3_11_DEFAULT)
    #define PRESCALER_TIMER2_3_11  64
  #endif
  #if defined(PWM_976HZ_TIMER2_3_11)
    #define PRESCALER_TIMER2_3_11  32
  #endif
  #if defined(PWM_3906HZ_TIMER2_3_11)
    #define PRESCALER_TIMER2_3_11  8
  #endif
  #if defined(PWM_31250HZ_TIMER2_3_11)
    #define PRESCALER_TIMER2_3_11  1
  #endif
#endif

//*********************************************************************************************************************
// Pin D0(RX) (328PB 16-bit Timer/Counter 3)
//*********************************************************************************************************************
//#define PWM_30HZ_TIMER3_0     1024
//#define PWM_122HZ_TIMER3_0    256
//#define PWM_488HZ_TIMER3_0_DEFAULT  64
//#define PWM_3906HZ_TIMER3_0   8
//#define PWM_31250HZ_TIMER3_0  1

//*********************************************************************************************************************
// Pin D1(TX) and D2 (328PB 16-bit Timer/Counter 4)
//*********************************************************************************************************************
//#define PWM_30HZ_TIMER4_1_2     1024
//#define PWM_122HZ_TIMER4_1_2    256
//#define PWM_488HZ_TIMER4_1_2_DEFAULT  64
//#define PWM_3906HZ_TIMER4_1_2   8
//#define PWM_31250HZ_TIMER4_1_2  1

//*********************************************************************************************************************
// Set PWM prescaler
//*********************************************************************************************************************
void set_PWM_prescaler(uint8_t pin, uint16_t prescale)
{
  byte mode;
  
  if (pin == 0 || pin == 1 || pin == 5 || pin == 6 || pin == 9 || pin == 10)
  {
    switch (prescale) // 8-bit Timer/Counter 0, 16-bit Timer/Counter 1, 328PB 16-bit Timer/Counter 3, 328PB 16-bit Timer/Counter 4
    {
      case    1: mode = 0b001; break;
      case    8: mode = 0b010; break;
      case   64: mode = 0b011; break;
      case  256: mode = 0b100; break;
      case 1024: mode = 0b101; break;
      default: return;
    }
  }
  else if (pin == 3 || pin == 11)
  {
    switch (prescale) // 8-bit Timer/Counter 2
    {
      case    1: mode = 0b001; break;
      case    8: mode = 0b010; break;
      case   32: mode = 0b011; break;
      case   64: mode = 0b100; break;
      case  128: mode = 0b101; break;
      case  256: mode = 0b110; break;
      case 1024: mode = 0b111; break;
      default: return;
    }
  }
  
  if (pin == 5 || pin == 6) // 8-bit Timer/Counter 0, PD5(5), PD6(6)
  {
    TCCR0B = (TCCR0B & 0b11111000) | mode;
  }
  else if (pin == 9 || pin == 10) // 16-bit Timer/Counter 1, PB1(9), PB2(10)
  {
    TCCR1B = (TCCR1B & 0b11111000) | mode;
  }
  else if (pin == 3 || pin == 11) // 8-bit Timer/Counter 2, PD3(3), PB3(11)
  {
    TCCR2B = (TCCR2B & 0b11111000) | mode;
  }
 #ifdef __AVR_ATmega328PB__
  else if (pin == 0) // 328PB 16-bit Timer/Counter 3, PD0(0)RX, not paired with specific pin PD2(2)
  {
    TCCR3B = (TCCR3B & 0b11111000) | mode;
  }
  else if (pin == 1) // 328PB 16-bit Timer/Counter 4, PD1(1)TX, paired with specific pin PD2(2)
  {
    TCCR4B = (TCCR4B & 0b11111000) | mode;
  }
 #endif
}

#endif // End __PWM_h__
 
