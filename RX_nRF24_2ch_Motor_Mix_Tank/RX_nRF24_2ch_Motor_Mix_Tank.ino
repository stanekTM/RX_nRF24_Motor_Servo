
//*********************************************************************************************************************************
// RC receiver 2ch (mix tank-arcade motor driver, telemetry)
//***********************************************************
// Simple RC receiver from my repository https://github.com/stanekTM/RX_nRF24_Motor_Servo/tree/master/RX_nRF24_2ch_Motor_Mix_Tank
//
// The hardware includes nRF24L01+ transceiver and ATmega328P processor.
// The firmware will be used for boats, tanks, and robots. The code is Arduino.
//
// Works with RC transmitters:
// TX_nRF24_2ch_OLED          https://github.com/stanekTM/TX_nRF24_2ch_OLED
// TX_nRF24_5ch_LED           https://github.com/stanekTM/TX_nRF24_5ch_LED
// OpenAVRc                   https://github.com/Ingwie/OpenAVRc_Dev
// Multiprotocol from my fork https://github.com/stanekTM/TX_FW_Multi_Stanek
//*********************************************************************************************************************************

#include <RF24.h>         // https://github.com/nRF24/RF24
//#include <printf.h>       // Print the radio debug info
#include <DigitalIO.h>    // https://github.com/greiman/DigitalIO
#include "PWMFrequency.h" // Used locally https://github.com/TheDIYGuy999/PWMFrequency


// Setting a unique address (5 bytes number or character)
const byte address[] = "jirka";

// RF communication channel settings (0-125, 2.4Ghz + 76 = 2.476Ghz)
#define RADIO_CHANNEL  76

// Setting the reaction of the motor to be rotated after the lever has been moved. Settings (0-255)
#define ACCELERATE_MOTOR_A  0
#define ACCELERATE_MOTOR_B  0

// Setting the maximum motor power in individual modes. Suitable, for example, when the motors
// powers are not the same and it is necessary to achieve straight driving. Settings (0-255)
#define MAX_FORW_MOTOR_A  255
#define MAX_BACK_MOTOR_A  255

#define MAX_FORW_MOTOR_B  255
#define MAX_BACK_MOTOR_B  255

// Brake setting, no brake 0, max brake 255. Settings (0-255)
#define BRAKE_MOTOR_A  255
#define BRAKE_MOTOR_B  255

// Alarm voltage setting
#define BATTERY_VOLTAGE    4.2
#define MONITORED_VOLTAGE  3.35

// Setting the dead zone of poor quality joysticks TX for the motor controller
#define DEAD_ZONE  15

// Setting the control range value
#define MIN_CONTROL_VAL  1000
#define MID_CONTROL_VAL  1500
#define MAX_CONTROL_VAL  2000

// Settings PWM (pin D5 or D6 are paired on timer0/8-bit, functions delay, millis, micros and delayMicroseconds)
// 1024 = 61Hz, 256 = 244Hz, 64 = 976Hz(default), 8 = 7812Hz
//#define PWM_MOTOR_A  64

// Settings PWM (pin D9 or D10 are paired on timer1/16-bit, Servo library)
// 1024 = 30Hz, 256 = 122Hz, 64 = 488Hz(default), 8 = 3906Hz
#define PWM_MOTOR_A  256

// Settings PWM (pin D3 or D11 are paired on timer2/8-bit, ServoTimer2 library)
// 1024 = 30Hz, 256 = 122Hz, 128 = 244Hz, 64 = 488Hz(default), 32 = 976Hz, 8 = 3906Hz
#define PWM_MOTOR_B  256

// Free pins
// Pin                     0
// Pin                     1
// Pin                     2
// Pin                     4
// Pin                     5
// Pin                     6
// Pin                     7
// Pin                     8
// Pin                     12 // MISO
// Pin                     13 // SCK
// Pin                     A6
 
// PWM pins for motor
#define PIN_PWM_1_MOTOR_A  9
#define PIN_PWM_2_MOTOR_A  10
#define PIN_PWM_3_MOTOR_B  3
#define PIN_PWM_4_MOTOR_B  11 // MOSI

// LED alarm
#define PIN_LED            A5

// Input battery
#define PIN_BATTERY        A7

// Pins for nRF24L01
#define PIN_CE             A0
#define PIN_CSN            A1

// Software SPI https://nrf24.github.io/RF24/md_docs_arduino.html
//----- SCK           16 - A2
//----- MOSI          17 - A3
//----- MISO          18 - A4

// Setting of CE and CSN pins
RF24 radio(PIN_CE, PIN_CSN);

//*********************************************************************************************************************
// This structure defines the data sent (max 32 bytes)
//*********************************************************************************************************************
struct rc_packet_size
{
  unsigned int ch_motorA = MID_CONTROL_VAL;
  unsigned int ch_motorB = MID_CONTROL_VAL;
};
rc_packet_size rc_packet;

//*********************************************************************************************************************
// This structure defines the received ACK payload data
//*********************************************************************************************************************
struct telemetry_packet_size
{
  byte rssi;    // Max 170 OpenTx!
  byte batt_A1 = 255;
  byte batt_A2; // Not used yet
};
telemetry_packet_size telemetry_packet;

//*********************************************************************************************************************
// Fail-safe, settings 1000-2000 (MIN_CONTROL_VAL = 1000, MID_CONTROL_VAL = 1500, MAX_CONTROL_VAL = 2000)
//*********************************************************************************************************************
void fail_safe()
{
  rc_packet.ch_motorA = MID_CONTROL_VAL;
  rc_packet.ch_motorB = MID_CONTROL_VAL;
}

//*********************************************************************************************************************
// Motor control
//*********************************************************************************************************************
int value_motorA = 0, value_motorB = 0;
int ch1 = 0, ch2 = 0;
int mix1 = 0, mix2 = 0;
int calc_mix = 258;

void motor_control()
{
  // mix tank-arcade
  ch1 = rc_packet.ch_motorA / 2;
  ch2 = rc_packet.ch_motorB / 2;
  mix1 = ch1 - ch2 + 1500;
  mix2 = ch1 + ch2;
  //Serial.println(mix1);
  
  // Forward motor A
  if (mix1 > MID_CONTROL_VAL + DEAD_ZONE)
  {
    value_motorA = map(mix1, MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL - calc_mix, ACCELERATE_MOTOR_A, MAX_FORW_MOTOR_A);
    value_motorA = constrain(value_motorA, ACCELERATE_MOTOR_A, MAX_FORW_MOTOR_A);
    analogWrite(PIN_PWM_2_MOTOR_A, value_motorA);
    digitalWrite(PIN_PWM_1_MOTOR_A, LOW);
    //Serial.println(value_motorA);
  }
  // Back motor A
  else if (mix1 < MID_CONTROL_VAL - DEAD_ZONE)
  {
    value_motorA = map(mix1, MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL + calc_mix, ACCELERATE_MOTOR_A, MAX_BACK_MOTOR_A);
    value_motorA = constrain(value_motorA, ACCELERATE_MOTOR_A, MAX_BACK_MOTOR_A);
    analogWrite(PIN_PWM_1_MOTOR_A, value_motorA);
    digitalWrite(PIN_PWM_2_MOTOR_A, LOW);
    //Serial.println(value_motorA);
  }
  else
  {
    analogWrite(PIN_PWM_1_MOTOR_A, BRAKE_MOTOR_A);
    analogWrite(PIN_PWM_2_MOTOR_A, BRAKE_MOTOR_A);
  }
  //Serial.println(value_motorA);
  
  // Forward motor B
  if (mix2 > MID_CONTROL_VAL + DEAD_ZONE)
  {
    value_motorB = map(mix2, MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL - calc_mix, ACCELERATE_MOTOR_B, MAX_FORW_MOTOR_B);
    value_motorB = constrain(value_motorB, ACCELERATE_MOTOR_B, MAX_FORW_MOTOR_B);
    analogWrite(PIN_PWM_4_MOTOR_B, value_motorB);
    digitalWrite(PIN_PWM_3_MOTOR_B, LOW);
    //Serial.println(value_motorB);
  }
  // Back motor B
  else if (mix2 < MID_CONTROL_VAL - DEAD_ZONE)
  {
    value_motorB = map(mix2, MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL + calc_mix, ACCELERATE_MOTOR_B, MAX_BACK_MOTOR_B);
    value_motorB = constrain(value_motorB, ACCELERATE_MOTOR_B, MAX_BACK_MOTOR_B);
    analogWrite(PIN_PWM_3_MOTOR_B, value_motorB);
    digitalWrite(PIN_PWM_4_MOTOR_B, LOW);
    //Serial.println(value_motorB);
  }
  else
  {
    analogWrite(PIN_PWM_3_MOTOR_B, BRAKE_MOTOR_B);
    analogWrite(PIN_PWM_4_MOTOR_B, BRAKE_MOTOR_B);
  }
  //Serial.println(value_motorB);
}

//*********************************************************************************************************************
// Program setup
//*********************************************************************************************************************
void setup()
{
  //Serial.begin(9600);
  //printf_begin(); // Print the radio debug info
  
  pinMode(PIN_PWM_1_MOTOR_A, OUTPUT);
  pinMode(PIN_PWM_2_MOTOR_A, OUTPUT);
  pinMode(PIN_PWM_3_MOTOR_B, OUTPUT);
  pinMode(PIN_PWM_4_MOTOR_B, OUTPUT);
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BATTERY, INPUT);
  
  fail_safe();
  
  // Setting the motor frequency
  setPWMPrescaler(PIN_PWM_1_MOTOR_A, PWM_MOTOR_A);
  setPWMPrescaler(PIN_PWM_3_MOTOR_B, PWM_MOTOR_B);
  
  // Define the radio communication
  radio.begin();
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(5, 5);
  radio.setChannel(RADIO_CHANNEL);
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
unsigned long rf_timeout = 0;
byte packet_counter = 0;
unsigned long packet_time = 0;

void send_and_receive_data()
{
  if (radio.available())
  {
    radio.read(&rc_packet, sizeof(rc_packet_size));

    radio.writeAckPayload(1, &telemetry_packet, sizeof(telemetry_packet_size));
    
    packet_counter++;
    
    rf_timeout = millis();
  }
  
  if (millis() - packet_time > 200)
  {
    packet_time = millis();
    //telemetry_packet.rssi = packet_counter;
    telemetry_packet.rssi = map(packet_counter, 0, 10, 0, 100);
    //telemetry_packet.rssi = constrain(telemetry_packet.rssi, 0, 100);
    packet_counter = 0;
  }
}

//*********************************************************************************************************************
// Battery voltage monitoring
//*********************************************************************************************************************
unsigned long adc_time = 0;
bool low_batt = 0; 
bool previous_state_batt;

void batt_monitoring()
{
  if (millis() - adc_time > 1000) // Delay ADC reading battery
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
 
