/*
  ***********************************************************************************************************************
  RC receiver 1 to 12 servo channels
  **********************************
  RC receiver from my repository https://github.com/stanekTM/RX_nRF24_Motor_Servo/tree/master/RX_nRF24_Xch_Servo
  
  Includes nRF24L01+ transceiver and ATmega328P/PB processor for PWM servo outputs and telemetry.
  
  Works with RC transmitters:
  TX_nRF24_2ch_OLED          https://github.com/stanekTM/TX_nRF24_2ch_OLED
  TX_nRF24_Xch_LED           https://github.com/stanekTM/TX_nRF24_Xch_LED
  OpenAVRc                   https://github.com/Ingwie/OpenAVRc_Dev
  Multiprotocol from my fork https://github.com/stanekTM/TX_FW_Multi_Stanek
  ***********************************************************************************************************************
*/

#include <RF24.h>      // v1.4.11
#include <RF24_config.h>
#include <nRF24L01.h>
//#include <printf.h>  // Print the radio debug info
#include <DigitalIO.h> // v1.0.1
#include <Servo.h>     // v1.2.2

// Setting a unique address (5 bytes number or character)
const byte address[] = "jirka";

// RF communication channel setting (0-125, 2.4Ghz + 76 = 2.476Ghz)
#define RF_CHANNEL  76

// Setting the number of servo channels (max. 12)
#define RC_CHANNELS  12

// Alarm voltage setting
#define BATTERY_VOLTAGE    4.2  // Maximum nominal battery voltage
#define MONITORED_VOLTAGE  3.45 // Minimum battery voltage for alarm

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

// Pins for servos (possible combination, max. 12)
const byte pins_servo[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

// LED alarm
#define PIN_LED       A5

// Input battery
#define PIN_BATTERY   A7

// Pins for nRF24L01+
#define PIN_CE        A0
#define PIN_CSN       A1

// Software SPI https://nrf24.github.io/RF24/md_docs_2arduino.html
//          SCK       A2
//          MOSI      A3
//          MISO      A4

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
// Attach servo pins
//*********************************************************************************************************************
Servo servo[RC_CHANNELS];
byte i;

void attach_servo_pins()
{
  for (i = 0; i < RC_CHANNELS; i++)
  {
    servo[i].attach(pins_servo[i]);
  }
}

//*********************************************************************************************************************
// Servo control
//*********************************************************************************************************************
void servo_control()
{
  for (i = 0; i < RC_CHANNELS; i++)
  {
    servo[i].writeMicroseconds(rc_packet[i]);
  }
  //Serial.println(rc_packet[0]);
}

//*********************************************************************************************************************
// Fail-safe, servo center position 1500
//*********************************************************************************************************************
void fail_safe()
{
  for (i = 0; i < RC_CHANNELS; i++)
  {
    rc_packet[i] = 1500;
  }
}

//*********************************************************************************************************************
// Program setup
//*********************************************************************************************************************
byte retry_delay = (RC_CHANNELS * 3) / 7;

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
  radio.setRetries(retry_delay, 0);
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
  
  servo_control();
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
    telemetry_packet.rssi = map(packet_counter, 267, 278, 0, 100);
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
 
