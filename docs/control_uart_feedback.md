# Control UART feedback packet

The firmware sends the binary feedback packet on every UART that enables
`FEEDBACK_SERIAL_USART2` or `FEEDBACK_SERIAL_USART3`. In the default
`VARIANT_USART` configuration, commands and feedback share USART3 at 115200 baud,
8 data bits, no parity, 1 stop bit.

A feedback packet is transmitted every 10 ms and begins with the little-endian
start frame `0xABCD`. The packet is a packed sequence of 16-bit fields in the
order shown below.

| Field | C type | Meaning |
| --- | --- | --- |
| `start` | `uint16_t` | Start frame, always `0xABCD` |
| `cmd1` | `int16_t` | Latest normalized input 1 command |
| `cmd2` | `int16_t` | Latest normalized input 2 command |
| `speedR_meas` | `int16_t` | Right motor speed in RPM |
| `speedL_meas` | `int16_t` | Left motor speed in RPM |
| `angleR_meas` | `int16_t` | Right wheel relative position in degrees, modulo 720 |
| `angleL_meas` | `int16_t` | Left wheel relative position in degrees, modulo 720 |
| `batVoltage` | `int16_t` | Calibrated battery voltage in firmware units |
| `boardTemp` | `int16_t` | Board temperature in degrees Celsius |
| `cmdLed` | `uint16_t` | Sideboard LED command/status byte carried as 16-bit |
| `checksum` | `uint16_t` | XOR of all previous packet fields |

## Reading wheel angle

`angleR_meas` and `angleL_meas` are relative wheel positions. Each wheel is set
to `0` degrees when the firmware boots. The interrupt handler then counts valid
Hall-sector transitions, so the value wraps over a two-turn window:

* forward motion from boot: `0, 4, 8, ... 716, 0, ...` on the stock 15 pole-pair
  motor;
* reverse motion from boot: `0, 716, 712, ...`.

The packet fields are little-endian. On Arduino-style hosts, update your feedback
structure to match the firmware structure exactly and update the checksum to
include both angle fields:

```c
typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  angleR_meas;
   int16_t  angleL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;

uint16_t checksum = feedback.start ^ feedback.cmd1 ^ feedback.cmd2 ^
                    feedback.speedR_meas ^ feedback.speedL_meas ^
                    feedback.angleR_meas ^ feedback.angleL_meas ^
                    feedback.batVoltage ^ feedback.boardTemp ^
                    feedback.cmdLed;
```

The `Arduino/hoverserial/hoverserial.ino` example already contains the updated
packet layout and prints the angle fields as values 5 and 6.
