# RC receiver 12ch (servo driver, telemetry)
Includes nRF24L01+ transceiver and ATmega328P/PB processor for PWM servo outputs and telemetry.

### Receiver specification:
* Operating Voltage: 3.3V - 6.0V (target typically 4.2V, 1S LiPo)

### Works with RC transmitters:
* [**TX_nRF24_2ch_OLED**](https://github.com/stanekTM/TX_nRF24_2ch_OLED)
* [**TX_nRF24_2ch_LED**](https://github.com/stanekTM/TX_nRF24_2ch_LED)
* [**OpenAVRc**](https://github.com/Ingwie/OpenAVRc_Dev)
* [**Multiprotocol**](https://github.com/stanekTM/TX_FW_Multi_Stanek) from my fork.

## Function
* Normal mode, LED is lit
* If the RX battery is low, the LED blink at 0.3s interval
* If we lose RF data for 1 second, the LED blink at 0.1s interval
* Fail-safe servos 1 -> 12 set to neutral or individually set in code

## Arduino pins
```
D2  - servo 1
D3  - servo 2
D4  - servo 3
D5  - servo 4
D6  - servo 5
D7  - servo 6
D8  - servo 7
D9  - servo 8
D10 - servo 9
D11 - servo 10
D12 - servo 11
D13 - servo 12

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
