# Arduino Nano 33 - Minimal Dimmer Implementation

This minimal hardware timer implementation works out of the box for all
Arduino Nano 33 boards:
* [Arduino Nano 33 IoT](https://docs.arduino.cc/hardware/nano-33-iot)
* [Arduino Nano 33 BLE](https://docs.arduino.cc/hardware/nano-33-ble)
* [Arduino Nano 33 BLE Sense](https://docs.arduino.cc/hardware/nano-33-ble-sense)

## Setup
Connect the Arduino to a suitable AC dimmer module, e.g. the RobotDyn® 4-channel AC dimmer:

| Arduino | Dimmer |   |
|---------|--------|---|
| GND     | GND    | R |
| 3.3V    | VCC    | R |
| D2      | Z-C    | R |
| D3      | D1     | O |
| D4      | D2     | O |
| D5      | D3     | O |
| D6      | D4     | O |

`R` indicates a required and `O` an optional connection.

## Usage
Comment out `#define NETWORK_FREQUENCY_50HZ` if your network frequency is 60Hz or leave it as is for 50Hz. Enable the desired lights using the `ENABLED_LIGHTS_MASK` bitmask, e.g.: 
```
#define ENABLED_LIGHTS_MASK (LIGHT_D3 | LIGHT_D4 | LIGHT_D5)
```
Set a light's brightness by calling:
```
set_brightness(&light_D4, 150);
```

> This implementation relies on `TIMER3_IRQn` or `TC3_IRQn` respectively so please make sure these timers are free.

> Do not add any `Serial.print()` calls to the interrupt service routines `zc_isr()` and `IRQ_HANDLER()` since this will most likely result in a stalled CPU.
