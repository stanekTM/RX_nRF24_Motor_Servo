# RC receiver 10ch (motor driver, servo driver, telemetry)
Includes nRF24L01+ transceiver, ATmega328P processor, 2x motor controller, servo outputs and telemetry.

The motor driver IC is based on MX1208, MX1508, MX1515, MX1616, MX1919 and others similar, which uses 4x PWM input control signals.
The option to adjust the brake is included in the code.

### Receiver specification:
* Operating Voltage: 3.3V - 6.0V (target typically 4.2V, 1S LiPo)
* Working current of the motor driver (MX1508): 1.5A (peak current up to 2.5A)

### Works with RC transmitters:
* [**TX_nRF24_2ch_OLED**](https://github.com/stanekTM/TX_nRF24_2ch_OLED)
* [**TX_nRF24_5ch_LED**](https://github.com/stanekTM/TX_nRF24_5ch_LED)
* [**OpenAVRc**](https://github.com/Ingwie/OpenAVRc_Dev)
* [**Multiprotocol**](https://github.com/stanekTM/TX_FW_Multi_Stanek) from my fork.

## Function
* Adjustable PWM of motor A and B
* Brake on, off or adjustable effect 
* Normal mode, LED is lit
* If the RX battery is low, the LED blink at 0.3s interval
* If we lose RF data for 1 second, the LED blink at 0.1s interval
* Fail-safe - motor A and B stop, servos 1 -> 8 set to neutral or individually set in code

## Arduino pins
```
D2  - servo 1
D4  - servo 2
D7  - servo 3
D8  - servo 4
D9  - servo 5
D10 - servo 6
D12 - servo 7
D13 - servo 8

D5  - pwm1/MotorA
D6  - pwm2/MotorA
D3  - pwm3/MotorB
D11 - pwm4/MotorB

A5  - LED
A7  - telemetry analog input RX battery

nRF24L01+:
A0  - CE
A1  - CSN
software SPI:
A2  - SCK
A3  - MOSI
A4  - MISO
```

## Used libraries
* <RF24.h>      https://github.com/nRF24/RF24
* <DigitalIO.h> https://github.com/greiman/DigitalIO
* <Servo.h> Arduino standard library
