# RC receiver 1 to 12 servo channels
Includes nRF24L01+ transceiver and ATmega328P/PB processor for PWM servo outputs and telemetry.

### Receiver specification:
* Operating Voltage: 3.3V - 6.0V (target typically 4.2V, 1S LiPo)

### Works with RC transmitters:
* [**TX_nRF24_2ch_OLED**](https://github.com/stanekTM/TX_nRF24_2ch_OLED)
* [**TX_nRF24_Xch_LED**](https://github.com/stanekTM/TX_nRF24_Xch_LED)
* [**OpenAVRc**](https://github.com/Ingwie/OpenAVRc_Dev)
* [**Multiprotocol**](https://github.com/stanekTM/TX_FW_Multi_Stanek) from my fork.

## Function
* Normal mode, LED is lit
* If the RX battery is low, the LED blink at 0.3s interval
* If we lose RF data for 1 second, the LED blink at 0.1s interval
* Fail-safe servos 1 -> 12 set to neutral or individually set in code

## Arduino pins
```
Servo pins:
Number of servos as needed (possible combination, max 12)
D2 - D13

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
