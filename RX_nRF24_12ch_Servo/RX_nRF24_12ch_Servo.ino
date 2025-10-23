
//*************************************************************************************************************************
// RC receiver 12ch (servo driver, telemetry)
//********************************************
// Simple RC receiver from my repository https://github.com/stanekTM/RX_nRF24_Motor_Servo/tree/master/RX_nRF24_12ch_Servo
//
// Works with RC transmitters:
// TX_nRF24_2ch_OLED          https://github.com/stanekTM/TX_nRF24_2ch_OLED
// TX_nRF24_5ch_LED           https://github.com/stanekTM/TX_nRF24_5ch_LED
// OpenAVRc                   https://github.com/Ingwie/OpenAVRc_Dev
// Multiprotocol from my fork https://github.com/stanekTM/TX_FW_Multi_Stanek
//*************************************************************************************************************************

#include <RF24.h>      // https://github.com/nRF24/RF24
//#include <printf.h>    // Print the radio debug info
#include <DigitalIO.h> // https://github.com/greiman/DigitalIO
#include <Servo.h>     // Arduino standard library


// Setting a unique address (5 bytes number or character)
const byte address[] = "jirka";

// RF communication channel settings (0-125, 2.4Ghz + 76 = 2.476Ghz)
#define RADIO_CHANNEL  76

// Alarm voltage setting
#define BATTERY_VOLTAGE    4.2
#define MONITORED_VOLTAGE  3.45

// Setting the control range value
#define MIN_CONTROL_VAL  1000
#define MID_CONTROL_VAL  1500
#define MAX_CONTROL_VAL  2000

// Free pins
// Pin                0
// Pin                1
// Pin                A6

// Pins for servos
#define PIN_SERVO_1   2
#define PIN_SERVO_2   3
#define PIN_SERVO_3   4
#define PIN_SERVO_4   5
#define PIN_SERVO_5   6
#define PIN_SERVO_6   7
#define PIN_SERVO_7   8
#define PIN_SERVO_8   9
#define PIN_SERVO_9   10
#define PIN_SERVO_10  11 // MOSI
#define PIN_SERVO_11  12 // MISO
#define PIN_SERVO_12  13 // SCK

// LED alarm
#define PIN_LED       A5

// Input battery
#define PIN_BATTERY   A7

// Pins for nRF24L01
#define PIN_CE        A0
#define PIN_CSN       A1

// Software SPI https://nrf24.github.io/RF24/md_docs_arduino.html
//----- SCK      16 - A2
//----- MOSI     17 - A3
//----- MISO     18 - A4

// Setting of CE and CSN pins
RF24 radio(PIN_CE, PIN_CSN);

//*********************************************************************************************************************
// This structure defines the data sent (max 32 bytes)
//*********************************************************************************************************************
struct rc_packet_size
{
  unsigned int ch_servo1  = MID_CONTROL_VAL;
  unsigned int ch_servo2  = MID_CONTROL_VAL;
  unsigned int ch_servo3  = MID_CONTROL_VAL;
  unsigned int ch_servo4  = MID_CONTROL_VAL;
  unsigned int ch_servo5  = MID_CONTROL_VAL;
  unsigned int ch_servo6  = MID_CONTROL_VAL;
  unsigned int ch_servo7  = MID_CONTROL_VAL;
  unsigned int ch_servo8  = MID_CONTROL_VAL;
  unsigned int ch_servo9  = MID_CONTROL_VAL;
  unsigned int ch_servo10 = MID_CONTROL_VAL;
  unsigned int ch_servo11 = MID_CONTROL_VAL;
  unsigned int ch_servo12 = MID_CONTROL_VAL;
};
rc_packet_size rc_packet;

//*********************************************************************************************************************
// This structure defines the received ACK payload data
//*********************************************************************************************************************
struct telemetry_packet_size
{
  byte rssi;
  byte batt_A1 = 255;
  byte batt_A2; // Not used yet
};
telemetry_packet_size telemetry_packet;

//*********************************************************************************************************************
// Fail-safe, settings 1000-2000 (MIN_CONTROL_VAL = 1000, MID_CONTROL_VAL = 1500, MAX_CONTROL_VAL = 2000)
//*********************************************************************************************************************
void fail_safe()
{
  rc_packet.ch_servo1  = MID_CONTROL_VAL;
  rc_packet.ch_servo2  = MID_CONTROL_VAL;
  rc_packet.ch_servo3  = MID_CONTROL_VAL;
  rc_packet.ch_servo4  = MID_CONTROL_VAL;
  rc_packet.ch_servo5  = MID_CONTROL_VAL;
  rc_packet.ch_servo6  = MID_CONTROL_VAL;
  rc_packet.ch_servo7  = MID_CONTROL_VAL;
  rc_packet.ch_servo8  = MID_CONTROL_VAL;
  rc_packet.ch_servo9  = MID_CONTROL_VAL;
  rc_packet.ch_servo10 = MID_CONTROL_VAL;
  rc_packet.ch_servo11 = MID_CONTROL_VAL;
  rc_packet.ch_servo12 = MID_CONTROL_VAL;
}

//*********************************************************************************************************************
// Attach servo pins
//*********************************************************************************************************************
Servo servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8, servo9, servo10, servo11, servo12;

void attach_servo_pins()
{
  servo1.attach(PIN_SERVO_1);
  servo2.attach(PIN_SERVO_2);
  servo3.attach(PIN_SERVO_3);
  servo4.attach(PIN_SERVO_4);
  servo5.attach(PIN_SERVO_5);
  servo6.attach(PIN_SERVO_6);
  servo7.attach(PIN_SERVO_7);
  servo8.attach(PIN_SERVO_8);
  servo9.attach(PIN_SERVO_9);
  servo10.attach(PIN_SERVO_10);
  servo11.attach(PIN_SERVO_11);
  servo12.attach(PIN_SERVO_12);
}

//*********************************************************************************************************************
// Servo control
//*********************************************************************************************************************
void servo_control()
{
  servo1.writeMicroseconds(rc_packet.ch_servo1);
  servo2.writeMicroseconds(rc_packet.ch_servo2);
  servo3.writeMicroseconds(rc_packet.ch_servo3);
  servo4.writeMicroseconds(rc_packet.ch_servo4);
  servo5.writeMicroseconds(rc_packet.ch_servo5);
  servo6.writeMicroseconds(rc_packet.ch_servo6);
  servo7.writeMicroseconds(rc_packet.ch_servo7);
  servo8.writeMicroseconds(rc_packet.ch_servo8);
  servo9.writeMicroseconds(rc_packet.ch_servo9);
  servo10.writeMicroseconds(rc_packet.ch_servo10);
  servo11.writeMicroseconds(rc_packet.ch_servo11);
  servo12.writeMicroseconds(rc_packet.ch_servo12);
  
  //Serial.println(rc_packet.ch_servo1);
}

//*********************************************************************************************************************
// Program setup
//*********************************************************************************************************************
void setup()
{
  //Serial.begin(9600);
  //printf_begin(); // Print the radio debug info
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BATTERY, INPUT);
  
  fail_safe();
  attach_servo_pins();
  
  // Define the radio communication
  radio.begin();
  radio.setAutoAck(1);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(0, 0);
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
  
  servo_control();
  send_and_receive_data();
  batt_monitoring();
  LED_mode();
}

//*********************************************************************************************************************
// Send and receive data
//*********************************************************************************************************************
byte packet_counter = 0;
unsigned long rf_timeout = 0;
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
  
  if (millis() - packet_time > 1000)
  {
    packet_time = millis();
    
    telemetry_packet.rssi = packet_counter;
    
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
 
