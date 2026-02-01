/*
  *******************************************************************************************************************************
  RC receiver 2 channels (mix tank-arcade motor driver)
  *****************************************************
  RC receiver from my repository https://github.com/stanekTM/RX_nRF24_Motor_Servo/tree/master/RX_nRF24_2ch_Motor_Mix_Tank
  
  Includes nRF24L01+ transceiver and ATmega328P/PB processor for PWM motor control and telemetry.
  
  Works with RC transmitters:
  TX_nRF24_2ch_OLED          https://github.com/stanekTM/TX_nRF24_2ch_OLED
  TX_nRF24_Xch_LED           https://github.com/stanekTM/TX_nRF24_Xch_LED
  OpenAVRc                   https://github.com/Ingwie/OpenAVRc_Dev
  Multiprotocol from my fork https://github.com/stanekTM/TX_FW_Multi_Stanek
  *******************************************************************************************************************************
*/

#include <RF24.h>      // v1.4.11
#include <RF24_config.h>
#include <nRF24L01.h>
//#include <printf.h>  // Print the radio debug info
#include <DigitalIO.h> // v1.0.1
#include "PWM_Frequency.h"

// Setting a unique address (5 bytes number or character)
const byte address[] = "jirka";

// RF communication channel setting (0-125, 2.4Ghz + 76 = 2.476Ghz)
#define RF_CHANNEL  76

// Setting the number of motor A and B channels (max. 2)
#define RC_CHANNELS  2

// Setting the reaction of the motor to be rotated after the lever has been moved. Settings (0-255)
#define ACCELERATE_MOTOR_A  0
#define ACCELERATE_MOTOR_B  0

// Setting the maximum motor power in individual modes. Suitable, for example, when the motors
// powers are not the same and it is necessary to achieve straight driving. Settings (0-255)
#define MAX_FORW_MOTOR_A  255
#define MAX_BACK_MOTOR_A  255

#define MAX_FORW_MOTOR_B  255
#define MAX_BACK_MOTOR_B  255

// Brake setting, no brake 0, maximum brake 255. Settings (0-255)
#define BRAKE_MOTOR_A  255
#define BRAKE_MOTOR_B  255

// Alarm voltage setting
#define BATTERY_VOLTAGE    4.2  // Maximum nominal battery voltage
#define MONITORED_VOLTAGE  3.45 // Minimum battery voltage for alarm

// Setting the dead zone of poor quality joysticks TX for the motor controller
#define DEAD_ZONE  15

// Setting the control range value
#define MIN_CONTROL_VAL  1000
#define MID_CONTROL_VAL  1500
#define MAX_CONTROL_VAL  2000

// Setting PWM
// Pin D5 and D6 (8-bit Timer/Counter 0, functions delay, millis, micros and delayMicroseconds)
// 1024 = 61Hz
// 256 = 244Hz
// 64 = 976Hz(default)
// 8 = 7812Hz
// 1 = 62500Hz
//#define PWM_MOTOR_A  64

// Pin D9 and D10 (16-bit Timer/Counter 1, Servo library)
// 1024 = 30Hz
// 256 = 122Hz
// 64 = 488Hz(default)
// 8 = 3906Hz
// 1 = 31250Hz
#define PWM_MOTOR_A  256

// Pin D3 and D11 (8-bit Timer/Counter 2, ServoTimer2, Tone library)
// 1024 = 30Hz
// 256 = 122Hz
// 128 = 244Hz
// 64 = 488Hz(default)
// 32 = 976Hz
// 8 = 3906Hz
// 1 = 31250Hz
#define PWM_MOTOR_B  256

// Pin D0(RX) (328PB 16-bit Timer/Counter 3)
// 1024 = 30Hz
// 256 = 122Hz
// 64 = 488Hz(default)
// 8 = 3906Hz
// 1 = 31250Hz
//#define PWM_MOTOR_A  64

// Pin D1(TX) and D2 (328PB 16-bit Timer/Counter 4)
// 1024 = 30Hz
// 256 = 122Hz
// 64 = 488Hz(default)
// 8 = 3906Hz
// 1 = 31250Hz
//#define PWM_MOTOR_B  64

// ATmega328P/PB pins overview
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

// PWM pins for motor A (possible combination, max. 2)
const byte pins_motorA[] = {9, 10};

// PWM pins for motor B (possible combination, max. 2)
const byte pins_motorB[] = {3, 11};

// LED alarm
#define PIN_LED            2

// Input battery
#define PIN_BATTERY        A7

// Pins for nRF24L01+
#define PIN_CE             A0
#define PIN_CSN            A1

// Software SPI https://nrf24.github.io/RF24/md_docs_2arduino.html
//          SCK            A2
//          MOSI           A3
//          MISO           A4

// nRF24 class driver
RF24 radio(PIN_CE, PIN_CSN);

//*********************************************************************************************************************
// Received data array (max. 32 bytes)
//*********************************************************************************************************************
unsigned int rc_packet[RC_CHANNELS] = {1500};
byte rc_packet_size = RC_CHANNELS * 2; // For one control channel with a value of 1000 to 2000 we need 2 bytes(packets)

//*********************************************************************************************************************
// Structure of sent ACK data
//*********************************************************************************************************************
struct telemetry_packet_size
{
  byte rssi;
  byte batt_A1 = 255;
  byte batt_A2; // Not used yet
};
telemetry_packet_size telemetry_packet;

//*********************************************************************************************************************
// Motor control
//*********************************************************************************************************************
int motorA_val = 0, motorB_val = 0;
int calc_mix = 258;

void motor_control()
{
  // Mix tank-arcade
  int ch1 = rc_packet[0] / 2;
  int ch2 = rc_packet[1] / 2;
  int mix1 = ch1 - ch2 + 1500;
  int mix2 = ch1 + ch2;
  //Serial.println(mix1);
  
  // Forward motor A
  if (mix1 > MID_CONTROL_VAL + DEAD_ZONE)
  {
    motorA_val = map(mix1, MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL - calc_mix, ACCELERATE_MOTOR_A, MAX_FORW_MOTOR_A);
    motorA_val = constrain(motorA_val, ACCELERATE_MOTOR_A, MAX_FORW_MOTOR_A);
    analogWrite(pins_motorA[1], motorA_val); 
    digitalWrite(pins_motorA[0], LOW);
    //Serial.println(motorA_val);
  }
  // Back motor A
  else if (mix1 < MID_CONTROL_VAL - DEAD_ZONE)
  {
    motorA_val = map(mix1, MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL + calc_mix, ACCELERATE_MOTOR_A, MAX_BACK_MOTOR_A);
    motorA_val = constrain(motorA_val, ACCELERATE_MOTOR_A, MAX_BACK_MOTOR_A);
    analogWrite(pins_motorA[0], motorA_val);
    digitalWrite(pins_motorA[1], LOW);
    //Serial.println(motorA_val);
  }
  else
  {
    analogWrite(pins_motorA[0], BRAKE_MOTOR_A);
    analogWrite(pins_motorA[1], BRAKE_MOTOR_A);
  }
  //Serial.println(motorA_val);
  
  // Forward motor B
  if (mix2 > MID_CONTROL_VAL + DEAD_ZONE)
  {
    motorB_val = map(mix2, MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL - calc_mix, ACCELERATE_MOTOR_B, MAX_FORW_MOTOR_B);
    motorB_val = constrain(motorB_val, ACCELERATE_MOTOR_B, MAX_FORW_MOTOR_B);
    analogWrite(pins_motorB[1], motorB_val);
    digitalWrite(pins_motorB[0], LOW);
    //Serial.println(motorB_val);
  }
  // Back motor B
  else if (mix2 < MID_CONTROL_VAL - DEAD_ZONE)
  {
    motorB_val = map(mix2, MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL + calc_mix, ACCELERATE_MOTOR_B, MAX_BACK_MOTOR_B);
    motorB_val = constrain(motorB_val, ACCELERATE_MOTOR_B, MAX_BACK_MOTOR_B);
    analogWrite(pins_motorB[0], motorB_val);
    digitalWrite(pins_motorB[1], LOW);
    //Serial.println(motorB_val);
  }
  else
  {
    analogWrite(pins_motorB[0], BRAKE_MOTOR_B);
    analogWrite(pins_motorB[1], BRAKE_MOTOR_B);
  }
  //Serial.println(motorB_val);
}

//*********************************************************************************************************************
// Fail-safe, motor neutral value 1500
//*********************************************************************************************************************
void fail_safe()
{
  for (byte i = 0; i < RC_CHANNELS; i++)
  {
    rc_packet[i] = 1500;
  }
}

//*********************************************************************************************************************
// Program setup
//*********************************************************************************************************************
void setup()
{
  //Serial.begin(9600);
  //printf_begin(); // Print the radio debug info
  
  pinMode(pins_motorA[0], OUTPUT);
  pinMode(pins_motorA[1], OUTPUT);
  pinMode(pins_motorB[0], OUTPUT);
  pinMode(pins_motorB[1], OUTPUT);
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BATTERY, INPUT);
  
  fail_safe();
  
  // Setting the motor frequency
  setPWMPrescaler(pins_motorA[0], PWM_MOTOR_A);
  setPWMPrescaler(pins_motorB[0], PWM_MOTOR_B);
  
  // Define the radio communication
  radio.begin();
  radio.setAutoAck(1);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(0, 0);
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN); // RF24_PA_MIN (-18dBm), RF24_PA_LOW (-12dBm), RF24_PA_HIGH (-6dbm), RF24_PA_MAX (0dBm)
  radio.openReadingPipe(1, address);
  radio.startListening();
}

//*********************************************************************************************************************
// Program loop
//*********************************************************************************************************************
void loop()
{
  //radio.printDetails();       // Smaller print raw register values
  //radio.printPrettyDetails(); // Larger print human readable data
  
  motor_control();
  send_and_receive_data();
  batt_monitoring();
  LED_mode();
}

//*********************************************************************************************************************
// Send and receive data
//*********************************************************************************************************************
unsigned int packet_counter = 0;
unsigned long rf_timeout = 0;
unsigned long packet_time = 0;

void send_and_receive_data()
{
  if (radio.available())
  {
    radio.read(&rc_packet, rc_packet_size);
    
    radio.writeAckPayload(1, &telemetry_packet, sizeof(telemetry_packet_size));
    
    packet_counter++;
    
    rf_timeout = millis();
  }
  
  if (millis() - packet_time > 1000)
  {
    packet_time = millis();
    telemetry_packet.rssi = map(packet_counter, 320, 333, 0, 100);
    telemetry_packet.rssi = constrain(telemetry_packet.rssi, 0, 100);
    //Serial.println(packet_counter);
    packet_counter = 0;
  }
}

//*********************************************************************************************************************
// Battery voltage monitoring
//*********************************************************************************************************************
unsigned long adc_time = 0;
bool low_batt = 0; 
bool previous_state_batt = 0;

void batt_monitoring()
{
  if (millis() - adc_time > 1000) // Battery reading delay
  {
    adc_time = millis();
    
    telemetry_packet.batt_A1 = map(analogRead(PIN_BATTERY), 0, 1023, 0, 255);
    
    low_batt = telemetry_packet.batt_A1 <= (255 / BATTERY_VOLTAGE) * MONITORED_VOLTAGE;
  }
  
  // Battery alarm lock
  if (low_batt)
  {
    previous_state_batt = 1;
  }
  low_batt = previous_state_batt;
  
  //Serial.println(low_batt);
}

//*********************************************************************************************************************
// LED blink mode and timeout for fail-safe
//*********************************************************************************************************************
void LED_mode()
{
  if (millis() - rf_timeout > 1000) // If we lose RF data for 1 second, the LED blink at 0.1s interval
  {
    fail_safe();
    
    blink(PIN_LED, 100);
  }
  else if (low_batt) // If the battery is low, the LED blink at 0.3s interval
  {
    blink(PIN_LED, 300);
  }
  else
  {
    digitalWrite(PIN_LED, HIGH); // Normal mode, LED is lit
  }
}

//*********************************************************************************************************************
// LED blink function
//*********************************************************************************************************************
unsigned long led_time = 0;
bool led_state;

void blink(uint8_t pin, uint16_t interval)
{
  if (millis() - led_time > interval)
  {
    led_time = millis();
    
    led_state = !led_state;
    
    digitalWrite(pin, led_state);
  }
}
 
