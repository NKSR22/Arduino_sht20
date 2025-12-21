# XY-MD04 (SHT20) Sensor with Arduino Mega 2560

This project reads Temperature and Humidity from an XY-MD04 Modbus RTU sensor using an Arduino Mega 2560 and a TTL to RS485 converter.

## Hardware Requirements

1.  **Arduino Mega 2560**
2.  **XY-MD04 Sensor** (SHT20 based Modbus RTU)
3.  **TTL to RS485 Module** (e.g., MAX485, HW-0519, etc.)
4.  Jumper Wires & Power Supply

## Wiring Connection

### Sensor to RS485 Module
| Sensor Wire Color | RS485 Module | Description       |
| ----------------- | ------------ | ----------------- |
| **Yellow**        | **A+**       | Data Line A       |
| **White**         | **B-**       | Data Line B       |
| **Red**           | **VCC**      | Power (5-30V DC)  |
| **Black**         | **GND**      | Ground            |

*> Note: Power the sensor with 5V from Arduino or external supply sharing GND.*

### RS485 Module to Arduino Mega 2560
| RS485 Module | Arduino Mega 2560 | Pin  | Description          |
| ------------ | ----------------- | ---- | -------------------- |
| **DI**       | **TX1**           | 18   | Serial1 Transmit     |
| **RO**       | **RX1**           | 19   | Serial1 Receive      |
| **DE**       | **Pin 2**         | 2    | Driver Enable        |
| **RE**       | **Pin 2**         | 2    | Receiver Enable      |
| **VCC**      | **5V**            | -    | 5V Power             |
| **GND**      | **GND**           | -    | Ground               |

*> **Important**: Connect DE and RE together to Pin 2. If your module has automatic flow control, Pin 2 connection is optional but the code still drives it harmlessly.*

## Software Setup

1.  **Platform**: PlatformIO
2.  **Framework**: Arduino
3.  **Board**: Arduino Mega 2560 (`megaatmega2560`)

## Usage

1.  Open the project in VS Code with PlatformIO extension.
2.  Connect the Arduino Mega via USB.
3.  Upload the firmware: `pio run -t upload`
4.  Open Serial Monitor: `pio device monitor` (Baudrate 9600).

## Expected Output

```
XY-MD04 Sensor Reader Started
Temperature: 28.50 C, Humidity: 60.20 %RH
Temperature: 28.50 C, Humidity: 60.10 %RH
...
```

## Troubleshooting
- **Timeout**: Check wiring, especially A/B polarity. Swap A and B if unsure. Check sensor power.
- **CRC Error**: Communication noise or incorrect baud rate (default is usually 9600).
