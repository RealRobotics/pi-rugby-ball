# Wiring Schedule

## PWM3901

The chosen drivers use the pins

| PWM3091 Pin | BCM Pin | Actual Pin |
|:-:|:-:|:-:|
| 3-5V | - | 17 |
| CS | 7 | 26 |
| SCK | 11 | 17 |
| MOSI | 10 | 17 |
| MISO | 9 | 17 |
| INT | 19 | 17 |
| GND | - | 25 |

## BNO085

The only drivers we have use I2C so that is what we use.

| BNO085 Pin | BCM Pin | Actual Pin |
|:----:|:-:|:-:|
| 3-5V | - | 1 |
| SDA  | 2 | 3 |
| SCL  | 3 | 5 |
| GND  | - | 6 |

No interrupt pin.
