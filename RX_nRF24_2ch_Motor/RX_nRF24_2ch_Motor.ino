/*
  *********************************************************************************************************************
  RC receiver 2 channels
  **********************
  RC receiver from my repository https://github.com/stanekTM/RX_nRF24_Motor_Servo/tree/master/RX_nRF24_2ch_Motor
  
  Includes nRF24L01+ transceiver and ATmega328P/PB processor for PWM motor control and telemetry.
  
  Works with RC transmitters:
  TX_nRF24_2ch_OLED          https://github.com/stanekTM/TX_nRF24_2ch_OLED
  TX_nRF24_Xch_LED           https://github.com/stanekTM/TX_nRF24_Xch_LED
  OpenAVRc                   https://github.com/Ingwie/OpenAVRc_Dev
  Multiprotocol from my fork https://github.com/stanekTM/TX_FW_Multi_Stanek
  *********************************************************************************************************************
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

// Setting the number of motor 1 and 2 channels (max. 2)
#define RC_CHANNELS  2

// Setting the reaction of the motor to be rotated after the lever has been moved. Settings (0-255)
#define ACCELERATE_MOTOR1  0
#define ACCELERATE_MOTOR2  0

// Setting the maximum motor power. Suitable for TX transmitters without endpoint setting. Settings (0-255)
#define MAX_FORW_MOTOR1  255
#define MAX_BACK_MOTOR1  255

#define MAX_FORW_MOTOR2  255
#define MAX_BACK_MOTOR2  255

// Brake setting, no brake 0, maximum brake 255. Settings (0-255)
#define BRAKE_MOTOR1  0
#define BRAKE_MOTOR2  0

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
//#define PWM_MOTOR1  64

// Pin D9 and D10 (16-bit Timer/Counter 1, Servo library)
// 1024 = 30Hz
// 256 = 122Hz
// 64 = 488Hz(default)
// 8 = 3906Hz
// 1 = 31250Hz
#define PWM_MOTOR1  64

// Pin D3 and D11 (8-bit Timer/Counter 2, ServoTimer2, Tone library)
// 1024 = 30Hz
// 256 = 122Hz
// 128 = 244Hz
// 64 = 488Hz(default)
// 32 = 976Hz
// 8 = 3906Hz
// 1 = 31250Hz
#define PWM_MOTOR2  64

// Pin D0(RX) (328PB 16-bit Timer/Counter 3)
// 1024 = 30Hz
// 256 = 122Hz
// 64 = 488Hz(default)
// 8 = 3906Hz
// 1 = 31250Hz
//#define PWM_MOTOR1  64

// Pin D1(TX) and D2 (328PB 16-bit Timer/Counter 4)
// 1024 = 30Hz
// 256 = 122Hz
// 64 = 488Hz(default)
// 8 = 3906Hz
// 1 = 31250Hz
//#define PWM_MOTOR2  64

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

// PWM pins for motor 1 (possible combination, max. 2)
const byte pins_motor1[] = {9, 10};

// PWM pins for motor 2 (possible combination, max. 2)
const byte pins_motor2[] = {3, 11};

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
int motor1_val = 0, motor2_val = 0;

void motor_control()
{
  // Forward motor 1
  if (rc_packet[0] > MID_CONTROL_VAL + DEAD_ZONE)
  {
    motor1_val = map(rc_packet[0], MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL, ACCELERATE_MOTOR1, MAX_FORW_MOTOR1);
    motor1_val = constrain(motor1_val, ACCELERATE_MOTOR1, MAX_FORW_MOTOR1);
    analogWrite(pins_motor1[1], motor1_val); 
    digitalWrite(pins_motor1[0], LOW);
  }
  // Back motor 1
  else if (rc_packet[0] < MID_CONTROL_VAL - DEAD_ZONE)
  {
    motor1_val = map(rc_packet[0], MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL, ACCELERATE_MOTOR1, MAX_BACK_MOTOR1);
    motor1_val = constrain(motor1_val, ACCELERATE_MOTOR1, MAX_BACK_MOTOR1);
    analogWrite(pins_motor1[0], motor1_val);
    digitalWrite(pins_motor1[1], LOW);
  }
  else
  {
    analogWrite(pins_motor1[0], BRAKE_MOTOR1);
    analogWrite(pins_motor1[1], BRAKE_MOTOR1);
  }
  //Serial.println(motor1_val);
  
  // Forward motor 2
  if (rc_packet[1] > MID_CONTROL_VAL + DEAD_ZONE)
  {
    motor2_val = map(rc_packet[1], MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL, ACCELERATE_MOTOR2, MAX_FORW_MOTOR2);
    motor2_val = constrain(motor2_val, ACCELERATE_MOTOR2, MAX_FORW_MOTOR2);
    analogWrite(pins_motor2[1], motor2_val);
    digitalWrite(pins_motor2[0], LOW);
  }
  // Back motor 2
  else if (rc_packet[1] < MID_CONTROL_VAL - DEAD_ZONE)
  {
    motor2_val = map(rc_packet[1], MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL, ACCELERATE_MOTOR2, MAX_BACK_MOTOR2);
    motor2_val = constrain(motor2_val, ACCELERATE_MOTOR2, MAX_BACK_MOTOR2);
    analogWrite(pins_motor2[0], motor2_val);
    digitalWrite(pins_motor2[1], LOW);
  }
  else
  {
    analogWrite(pins_motor2[0], BRAKE_MOTOR2);
    analogWrite(pins_motor2[1], BRAKE_MOTOR2);
  }
  //Serial.println(motor2_val);
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
  
  pinMode(pins_motor1[0], OUTPUT);
  pinMode(pins_motor1[1], OUTPUT);
  pinMode(pins_motor2[0], OUTPUT);
  pinMode(pins_motor2[1], OUTPUT);
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BATTERY, INPUT);
  
  fail_safe();
  
  // Setting the motor frequency
  setPWMPrescaler(pins_motor1[0], PWM_MOTOR1);
  setPWMPrescaler(pins_motor2[0], PWM_MOTOR2);
  
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
 
