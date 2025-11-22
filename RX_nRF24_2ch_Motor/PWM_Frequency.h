/*
  ****************************************************************************
  PWM frequency prescaler
  ***********************
  Thanks to the original author "TheDIYGuy999" https://github.com/TheDIYGuy999
  ****************************************************************************
*/

#ifndef PWM_Frequency_h
#define PWM_Frequency_h

#include <Arduino.h>


/*
  ***************************
  ATmega328P and ATmega328PB
  ***************************
  Pin D5 and D6 (8-bit Timer/Counter 0, functions delay, millis, micros and delayMicroseconds)
  1024 = 61Hz
  256 = 244Hz
  64 = 976Hz(default)
  8 = 7812Hz
  1 = 62500Hz
  
  Pin D9 and D10 (16-bit Timer/Counter 1, Servo library)
  1024 = 30Hz
  256 = 122Hz
  64 = 488Hz(default)
  8 = 3906Hz
  1 = 31250Hz
  
  Pin D3 and D11 (8-bit Timer/Counter 2, ServoTimer2, Tone library)
  1024 = 30Hz
  256 = 122Hz
  128 = 244Hz
  64 = 488Hz(default)
  32 = 976Hz
  8 = 3906Hz
  1 = 31250Hz
  
  Pin D0(RX) (328PB 16-bit Timer/Counter 3)
  1024 = 30Hz
  256 = 122Hz
  64 = 488Hz(default)
  8 = 3906Hz
  1 = 31250Hz
  
  Pin D1(TX) and D2 (328PB 16-bit Timer/Counter 4)
  1024 = 30Hz
  256 = 122Hz
  64 = 488Hz(default)
  8 = 3906Hz
  1 = 31250Hz
*/
void setPWMPrescaler(uint8_t pin, uint16_t prescale)
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
#ifdef ATmega328PB
  else if (pin == 0) // 328PB 16-bit Timer/Counter 3, PD0(0)RX, unused specific pin PD2(2)
  {
    TCCR3B = (TCCR3B & 0b11111000) | mode;
  }
  else if (pin == 1) // 328PB 16-bit Timer/Counter 4, PD1(1)TX, unused specific pin PD2(2)
  {
    TCCR4B = (TCCR4B & 0b11111000) | mode;
  }
#endif
}

#endif
 
