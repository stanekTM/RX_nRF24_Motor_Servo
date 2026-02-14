/*
  *****************************************************************************************************************************
  RC receiver with outputs for motors, servos and telemetry
  *********************************************************
  RC receiver from my repository https://github.com/stanekTM/RX_nRF24_Stanek
  
  Hardware includes nRF24L01+ transceiver, ATmega328P/PB processor and motor driver.
  
  Works with RC transmitters:
  TX_nRF24_2ch_OLED             https://github.com/stanekTM/TX_nRF24_2ch_OLED
  TX_nRF24_Xch_LED              https://github.com/stanekTM/TX_nRF24_Xch_LED
  OpenAVRc                      https://github.com/Ingwie/OpenAVRc_Dev
  Multiprotocol TX from my fork https://github.com/stanekTM/TX_FW_Multi_Stanek
  *****************************************************************************************************************************
*/

#include <RF24.h>      // v1.4.11
#include <RF24_config.h>
#include <nRF24L01.h>
//#include <printf.h>  // Print the radio debug info
#include <DigitalIO.h> // v1.0.1
#include <Servo.h>     // v1.2.2
#include "PWM.h"
#include "Pins.h"
#include "Config.h"

//*********************************************************************************************************************
// Program setup
//*********************************************************************************************************************
void setup()
{
  //Serial.begin(9600);
  //printf_begin(); // Print the radio debug info
  
  radio_setup();
  
  servo_setup();
  motor_setup();
  
  fail_safe();
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BATTERY, INPUT);
}

//*********************************************************************************************************************
// Program loop
//*********************************************************************************************************************
void loop()
{
  //radio.printDetails();       // Smaller print raw register values
  //radio.printPrettyDetails(); // Larger print human readable data
  
  send_and_receive_data();
  
  servo_control();
  motor_control();
  
  batt_monitoring();
  LED_mode();
}
 
