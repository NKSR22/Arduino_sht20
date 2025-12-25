#include <Arduino.h>

/**
 * XY-MD04 (SHT20) Temperature and Humidity Sensor via Modbus RTU (RS485)
 *
 * Hardware:
 * - Arduino Mega 2560
 * - TTL to RS485 Module (e.g., MAX485 or generic V2.2)
 * - XY-MD04 Sensor
 *
 * Wiring:
 *
 * RS485 Module <-> Arduino Mega 2560
 * VCC          <-> 5V
 * GND          <-> GND
 * DI (TX)      <-> TX1 (Pin 18)
 * RO (RX)      <-> RX1 (Pin 19)
 * DE/RE        <-> Pin 2 (Control Pin)
 *
 * RS485 Module <-> XY-MD04 Sensor
 * A+           <-> A+ (Yellow)
 * B-           <-> B- (White)
 * GND          <-> Black
 * VCC          <-> Red (5-30V)
 */

// #define RS485_DE_RE 2  // Control pin removed for Auto-RS485
#define RS485_TX 18
#define RS485_RX 19

// Hardware Serial 1 for RS485
#define ModbusSerial Serial1

// Buffer for sending/receiving Modbus frames
uint8_t requestFrame[] = {
    0x01, 0x04, 0x00, 0x01,
    0x00, 0x02, 0x20, 0x0B}; // Read Temp & Humi Input Registers (0x04)
// Command Analysis:
// 0x01: Slave Address
// 0x04: Function Code (Read Input Registers) - Correct for XY-MD04
// 0x00 0x01: Start Address (Temperature)
// 0x00 0x02: Quantity (2 Registers: Temp & Humi)
// 0x20 0x0B: CRC Checksum (Low Byte First)

void setup() {
  // Initialize Debug Serial
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  Serial.println("XY-MD04 Sensor Reader Started");

  // Initialize Modbus Serial
  ModbusSerial.begin(9600); // Sensor default is 9600, 8, N, 1
}

uint16_t calculateCRC(uint8_t *buffer, int length) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void loop() {
  // 1. Send Request
  // Clear buffer before sending to avoid reading echo/noise
  while (ModbusSerial.available() > 0) {
    ModbusSerial.read();
  }

  ModbusSerial.write(requestFrame, sizeof(requestFrame));
  // ModbusSerial.flush(); // Wait for transmission to complete (optional for
  // Auto logic but good practice)

  // 2. Wait for Response
  // Expected response length:
  // Slave(1) + Func(1) + ByteCount(1) + Temp(2) + Humi(2) + CRC(2) = 9 bytes
  uint32_t startTime = millis();
  while (ModbusSerial.available() < 9 && millis() - startTime < 1000) {
    delay(10);
  }

  if (ModbusSerial.available() >= 9) {
    uint8_t response[9];
    Serial.print("RX: ");
    for (int i = 0; i < 9; i++) {
      response[i] = ModbusSerial.read();
      if (response[i] < 0x10)
        Serial.print("0");
      Serial.print(response[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Optional: Clear remaining buffer
    while (ModbusSerial.available())
      ModbusSerial.read();

    // 3. Parse Data
    // Check Address and Function Code
    // Response should start with 0x01 0x04
    if (response[0] == 0x01 && response[1] == 0x04) {

      // Calculate CRC to verify
      uint16_t calCrc = calculateCRC(response, 7);
      uint16_t sentCrcCombined = response[7] | (response[8] << 8);

      if (calCrc == sentCrcCombined) {
        int16_t rawTemp = (response[3] << 8) | response[4];
        int16_t rawHumi = (response[5] << 8) | response[6];

        float temperature = rawTemp / 10.0;
        float humidity = rawHumi / 10.0;

        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" C, Humidity: ");
        Serial.print(humidity);
        Serial.println(" %RH");
      } else {
        Serial.print("CRC Error. Calc: ");
        Serial.print(calCrc, HEX);
        Serial.print(" Recv: ");
        Serial.println(sentCrcCombined, HEX);
      }

    } else {
      Serial.println("Invalid Response Header");
    }

  } else {
    if (ModbusSerial.available() > 0) {
      Serial.print("Incomplete RX: ");
      while (ModbusSerial.available()) {
        uint8_t b = ModbusSerial.read();
        if (b < 0x10)
          Serial.print("0");
        Serial.print(b, HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("Timeout - No data");
    }
  }

  delay(2000); // Read every 2 seconds
}
